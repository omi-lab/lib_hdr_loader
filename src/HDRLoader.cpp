// See the original_src folder for the original code.
//
// Read HDR code is partially derrived from here:
// https://www.flipcode.com/archives/HDR_Image_Reader.shtml
//
// Write HDR code is partially derrived from here:
// https://github.com/Allda/HDRlib/blob/master/HDRlib/rgbe.cpp
// And uses the following license: 
//
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "lib_hdr_loader/HDRLoader.h"

#include <cmath>
#include <cstring>
#include <cassert>
#include <iostream>
#include <atomic>

#include "tp_utils/Parallel.h"

namespace lib_hdr_loader
{

namespace
{
#define MINELEN	     8      // Min scanline length for encoding.
#define MAXELEN	     0x7fff // Max scanline length for encoding.
#define MINRUNLENGTH 4      // Min RLE run length.

//##################################################################################################
bool oldDecrunch(std::istream& hdrStream, uint8_t* scanline, int len, std::string& errorMessage)
{
  int rshift = 0;
  while(len > 0)
  {
    scanline[0] = hdrStream.get();
    scanline[1] = hdrStream.get();
    scanline[2] = hdrStream.get();
    scanline[3] = hdrStream.get();

    if(hdrStream.eof())
    {
      errorMessage = "Unexpected EOF!";
      return false;
    }

    if(scanline[0] == 1 && scanline[1] == 1 && scanline[2] == 1)
    {
      for(int i = scanline[3] << rshift; i>0; i--)
      {
        std::memcpy(&scanline[0], &scanline[-4], 4);
        scanline+=4;
        len--;
      }
      rshift += 8;
    }
    else
    {
      scanline+=4;
      len--;
      rshift = 0;
    }
  }

  return true;
}

//##################################################################################################
bool decrunch(std::istream& hdrStream, uint8_t* scanline, int len, std::string& errorMessage)
{
  if (len < MINELEN || len > MAXELEN)
    return oldDecrunch(hdrStream, scanline, len, errorMessage);

  if(hdrStream.get() != 2)
  {
    hdrStream.seekg(-1, std::ios::cur);
    return oldDecrunch(hdrStream, scanline, len, errorMessage);
  }

  scanline[1] = hdrStream.get();
  scanline[2] = hdrStream.get();
  int i = hdrStream.get();
  if (scanline[1] != 2 || scanline[2] & 128)
  {
    scanline[0] = 2;
    scanline[3] = i;
    return oldDecrunch(hdrStream, scanline + 4, len - 1, errorMessage);
  }

  // read each component
  for(int i = 0; i < 4; i++)
  {
    int const len4 = (len << 2) + i;
    for(int j = i; j < len4;)
    {
      uint8_t code = hdrStream.get();
      if(code > 128) // run
      {
        code &= 127;
        uint8_t val = hdrStream.get();
        for(auto k=0; k<code; ++k, j+=4)
          scanline[j]=val;
      }
      else
      {
        char buf[128];
        hdrStream.read(buf, code);
        for(auto k=0; k<code; ++k, j+=4)
          scanline[j]=static_cast<uint8_t>(buf[k]);
      }
    }
  }

  if(hdrStream.eof())
  {
    errorMessage = "Unexpected EOF in decrunch.";
    return false;
  }
  return true;
}

#if 0
// not used
//##################################################################################################
bool rle_old(std::ostream& hdrStream, const uint8_t* scanline, int len)
{
  auto data = [&](int i)
  {
    return char(scanline[i*4]);
  };

  for(int cur=0; cur<len;)
  {
    int begRun = cur;

    // Look for a run.
    int runCount = 0;
    int oldRunCount = 0;
    while((runCount<MINRUNLENGTH) && (begRun<len))
    {
      begRun += runCount;
      oldRunCount = runCount;
      runCount = 1;
      while(((begRun+runCount) < len) && (runCount<127)  && (data(begRun) == data(begRun + runCount)))
        runCount++;
    }

    // If data before next big run is a short run then write it as such
    if((oldRunCount>1)&&(oldRunCount == begRun - cur))
    {
      //write short run
      hdrStream << char(128 + oldRunCount);
      hdrStream << data(cur);
      cur = begRun;
    }

    //Write out bytes until we reach the start of the next run
    while(cur < begRun)
    {
      int nonrunCount = begRun - cur;
      if (nonrunCount > 128)
        nonrunCount = 128;

      hdrStream << char(nonrunCount);

      {
         const uint8_t* s = scanline+(cur*4);
         const uint8_t* sMax = s+(nonrunCount*4);
         for(; s<sMax; s+=4)
           hdrStream << char(*s);
      }

      cur += nonrunCount;
    }

    // Write out next run if one was found
    if (runCount >= MINRUNLENGTH)
    {
      hdrStream << char(128 + runCount);
      hdrStream << data(begRun);
      cur += runCount;
    }

    if(hdrStream.fail())
      return false;
  }

  return true;
}
#endif

//##################################################################################################
bool rle(std::ostream& hdrStream, const uint8_t* scanline, int len_)
{
  // checking constrain on the length of the scan line 600*127/2 = 38100 pixels
  // buffer size shall satisfy extreme case when whole line is constant
  if(len_> 600*127/2)
    return false;

  char lastSymbol = char(scanline[0]);
  int longRunPosition = 0;
  int nonRunPosition = 0;
  int len4 = len_ << 2;
  const int MINRUNLENGTH2 = 3 << 2; // compressing symbol sequence with 3 and more repetitions

  char buf[600];

  for(int cur=0; cur < len4; cur += 4 )
  {
    if(lastSymbol == char(scanline[cur])){
      continue;
    }

    if((cur - longRunPosition) < MINRUNLENGTH2){
      longRunPosition = cur;
      lastSymbol = char(scanline[cur]);
      continue;
    }

    if(int shortRunSize = (longRunPosition - nonRunPosition) >> 2 ; shortRunSize > 0 ){
      int counter = nonRunPosition;
      do{
        int packSize = std::min(shortRunSize, 128);
        buf[0] = char(packSize);
        for(auto i = 1; i <= packSize; i++, counter+=4)
          buf[i] = char(scanline[counter]);
        hdrStream.write(buf, packSize + 1);
        shortRunSize -= packSize;
      }while(shortRunSize != 0);
    }

    if(auto longRunSize = (cur - longRunPosition) >> 2; longRunSize > 0 ){
      int counter = 0;
      do{
        int packSize = std::min(longRunSize, 127);
        buf[counter++] = char(128 + packSize);
        buf[counter++] = lastSymbol;
        longRunSize -= packSize;
      }while(longRunSize != 0);
      hdrStream.write(buf, counter);
    }

    lastSymbol = char(scanline[cur]);
    longRunPosition = nonRunPosition = cur;
  }

  // repeating code to process tail of the scan line
  if((len4 - longRunPosition) < MINRUNLENGTH2)
        longRunPosition = len4;

  if(int shortRunSize = (longRunPosition - nonRunPosition) >> 2 ; shortRunSize > 0 ){
    int counter = nonRunPosition;
    do{
      int packSize = std::min(shortRunSize, 128);
      buf[0] = char(packSize);
      for(auto i = 1; i <= packSize; i++, counter+=4)
        buf[i] = char(scanline[counter]);
      hdrStream.write(buf, packSize + 1);
      shortRunSize -= packSize;
    }while(shortRunSize != 0);
  }

  if(auto longRunSize = (len4 - longRunPosition) >> 2; longRunSize > 0 ){
    int counter = 0;
    do{
      int packSize = std::min(longRunSize, 127);
      buf[counter++] = char(128 + packSize);
      buf[counter++] = lastSymbol;
      longRunSize -= packSize;
    }while(longRunSize != 0);
    hdrStream.write(buf, counter);
  }

  return true;
}


//##################################################################################################
bool crunch(std::ostream& hdrStream, const uint8_t* scanline, int len)
{
  hdrStream << char(2) << char(2) << char((len>>8)&0xFF) << char(len&0xFF);
  for(size_t s=0; s<4; s++)
    if(!rle(hdrStream, scanline+s, len))
      return false;
  return true;
}

}

//##################################################################################################
bool loadHDRToRGBE(std::istream& hdrStream, const std::function<uint8_t*(size_t w, size_t h, const HDRHeader&)>& getImageBuffer, std::string& errorMessage)
{
  assert(hdrStream.flags() | std::ios_base::binary);

  auto fail = [&](const auto& msg)
  {
    if(!errorMessage.empty())
      errorMessage += '\n';
    errorMessage += msg;
    return false;
  };

  HDRHeader header;

  // Parse the program type header. Looking for one of the following:
  //   "#?RGBE\n"
  //   "#?RADIANCE\n"
  {
    char str[16];
    for(size_t i=0; ; i++)
    {
      if(i==16)
        return fail("Program type section too long.");

      int c = hdrStream.get();
      if(c == std::istream::traits_type::eof())
        return fail("EOF reached while reading program type.");

      str[i] = c;

      if(c == '\n')
      {
        if(i==6 && std::memcmp(str, "#?RGBE", 6)==0)
        {
          header.programType = "RGBE";
          break;
        }

        if(i==10 && std::memcmp(str, "#?RADIANCE", 10)==0)
        {
          header.programType = "RADIANCE";
          break;
        }

        return fail("Incorrect header.");
      }
    }
  }

  // Read the comment
  for(char oldc=0; ; )
  {
    int c = hdrStream.get();
    if(c == std::istream::traits_type::eof())
      return fail("EOF reached while reading comment.");

    if(c == '\n' && oldc == '\n')
      break;

    oldc = c;
  }

  char reso[200];
  for(size_t i=0; ; i++)
  {
    if(i==200)
      return fail("Resolution section too long.");

    int c = hdrStream.get();
    if(c == std::istream::traits_type::eof())
      return fail("EOF reached while reading resolution.");

    reso[i] = c;

    if(c == '\n')
      break;
  }

  size_t w=0;
  size_t h=0;

  //This is not complete X and Y can be swapped to rotate the image and both can be either + or - to flip.
  if(!std::sscanf(reso, "-Y %zu +X %zu", &h, &w))
    return fail("Failed to parse resolution.");

  if(w<1 || w>16384 || h<1 || h>16384)
    return fail("Invalid resolution.");

  uint8_t* buffer = getImageBuffer(w, h, header);
  if(!buffer)
    return fail("Failed to get image buffer.");

  size_t stride = w*4;
  uint8_t* scanline = buffer;
  uint8_t* scanlineMax = scanline + stride*h;
  for(; scanline<scanlineMax; scanline+=stride)
    if(!decrunch(hdrStream, scanline, int(w), errorMessage))
      return fail("Decompression failed.");

  return true;
}

//##################################################################################################
bool saveRGBEToHDR(std::ostream& hdrStream, const uint8_t* buffer, size_t w, size_t h, const HDRHeader& header, std::string& errorMessage)
{
  assert(hdrStream.flags() | std::ios_base::binary);

  auto fail = [&](const auto& msg)
  {
    errorMessage = msg;
    return false;
  };

  // Write the header out.
  hdrStream << "#?RADIANCE";
  hdrStream << '\n';
  hdrStream << "# " << header.comment << '\n';
  hdrStream << "FORMAT=" << header.format << '\n';
  hdrStream << '\n';
  hdrStream << "-Y " << h << " +X " << w;
  hdrStream << '\n';

  if(hdrStream.fail())
    return fail("Failed to write header.");

  // The scanline length is not compatible with RLE so write without RLE.
  if(w<MINELEN || w>MAXELEN)
  {
    hdrStream.write(reinterpret_cast<const char*>(buffer), size_t(w*h*4));
    if(!hdrStream.bad())
      return fail("Failed to write without RLE.");
    return true;
  }

  // Write out the RLE RGBE data
  size_t stride = w*4;
#ifdef TP_NO_THREADS
  std::vector<std::stringstream> streams(1);
#else
  std::vector<std::stringstream> streams(std::thread::hardware_concurrency());
#endif
  std::atomic<std::size_t> line {0};
  std::atomic<std::size_t> streamCount{0};
  std::atomic<std::size_t> max_line_size{0};
  std::vector<std::tuple<int,int,int>> streamStartEnd(h);
  bool failFlag = false;
  tp_utils::parallel([&](auto /*locker*/)
  {
    auto sc = streamCount++;
    auto& stream = streams[sc];
    for(;;)
    {
      auto i = line++;
      if(i>=h || failFlag)
        break;
      auto start = stream.tellp();
      auto s = buffer + stride*i;
      if(!crunch(stream, s, int(w))){
        failFlag = true;
        break;
      }
      streamStartEnd[i] = {int(sc), int(start), int(stream.tellp())};
      size_t size = stream.tellp() - start;

      size_t prev_value = max_line_size;
      while(prev_value < size &&
            !max_line_size.compare_exchange_weak(prev_value, size));
    }
  });

  if(failFlag)
    return  fail("Compression failed.");

  std::unique_ptr<char[]> buff(new char[max_line_size]);
  for(auto const& i: streamStartEnd){
    streams[std::get<0>(i)].seekg(std::get<1>(i));
    auto size = std::get<2>(i)-std::get<1>(i);
    streams[std::get<0>(i)].read(buff.get(), size);
    hdrStream.write(buff.get(), size);
  }

  return true;
}

}
