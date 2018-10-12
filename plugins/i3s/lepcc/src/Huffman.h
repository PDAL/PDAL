/*
Copyright 2016 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A local copy of the license and additional notices are located with the
source distribution at:

http://github.com/Esri/lepcc/

Contributors:  Thomas Maurer
*/

#ifndef HUFFMAN_H
#define HUFFMAN_H

#include <vector>
#include <cstring>
#include <utility>
#include "lepcc_types.h"

namespace lepcc
{
  class Huffman
  {
  public:
    Huffman() : m_maxHistoSize(1 << 15), m_maxNumBitsLUT(12), m_numBitsToSkipInTree(0), m_root(0) {};
    ~Huffman() { Clear(); };

    // Limitation: We limit the max Huffman code length to 32 bit. If this happens, the function ComputeCodes() 
    // returns false. In that case don't use Huffman coding but Lerc only instead. 
    // This won't happen easily. For the worst case input maximizing the Huffman code length the counts in the 
    // histogram have to follow the Fibonacci sequence. Even then, for < 9,227,465 data values, 32 bit is
    // the max Huffman code length possible. 

    int64 ComputeNumBytesNeededToEncode(const std::vector<int>& histo);

    // dst buffer is already allocated. byte ptr is moved like a file pointer.
    bool Encode(Byte** ppByte, int64 bufferSize, const std::vector<Byte>& dataVec) const;

    // resize dataVec before call
    bool Decode(const Byte** ppByte, int64 bufferSize, std::vector<Byte>& dataVec);

    void Clear();

  private:

    bool ComputeCodes(const std::vector<int>& histo);    // input histogram, size < 2^15
    bool ComputeCompressedSize(const std::vector<int>& histo, int& numBytes, double& avgBpp) const;

    bool WriteCodeTable(Byte** ppByte) const;
    bool ReadCodeTable(const Byte** ppByte, int lerc2Version = 3);

    bool BuildTreeFromCodes(int& numBitsLUT);
    bool DecodeOneValue(const unsigned int** ppSrc, int& bitPos, int numBitsLUT, int& value) const;

  private:

    struct Node
    {
      int weight;
      short value;
      Node *child0, *child1;

      Node(short val, int cnt)    // new leaf node for val
      {
        value = val;
        weight = -cnt;
        child0 = child1 = 0;
      }

      Node(Node* c0, Node* c1)    // new internal node from children c0 and c1
      {
        value = -1;
        weight = c0->weight + c1->weight;
        child0 = c0;
        child1 = c1;
      }

      bool operator < (const Node& other) const  { return weight < other.weight; }

      bool TreeToLUT(unsigned short numBits, unsigned int bits, std::vector<std::pair<unsigned short, unsigned int> >& luTable) const
      {
        if (child0)
        {
          if (numBits == 32    // the max huffman code length we allow
            || !child0->TreeToLUT(numBits + 1, (bits << 1) + 0, luTable)
            || !child1->TreeToLUT(numBits + 1, (bits << 1) + 1, luTable))
          {
            return false;
          }
        }
        else
          luTable[value] = std::pair<unsigned short, unsigned int>(numBits, bits);

        return true;
      }

      void FreeTree(int& n)
      {
        if (child0)
        {
          child0->FreeTree(n);
          delete child0;
          child0 = 0;
          n--;
        }
        if (child1)
        {
          child1->FreeTree(n);
          delete child1;
          child1 = 0;
          n--;
        }
      }
    };

  private:

    size_t m_maxHistoSize;
    std::vector<std::pair<unsigned short, unsigned int> > m_codeTable;
    std::vector<std::pair<short, short> > m_decodeLUT;
    int m_maxNumBitsLUT;
    int m_numBitsToSkipInTree;
    Node* m_root;

    static int GetIndexWrapAround(int i, int size)  { return i - (i < size ? 0 : size); }

    bool ComputeNumBytesCodeTable(int& numBytes) const;
    bool GetRange(int& i0, int& i1, int& maxCodeLength) const;
    bool BitStuffCodes(Byte** ppByte, int i0, int i1) const;
    bool BitUnStuffCodes(const Byte** ppByte, int i0, int i1);
    bool ConvertCodesToCanonical();
    void ClearTree();
  };

  // -------------------------------------------------------------------------- ;

  inline bool Huffman::DecodeOneValue(const unsigned int** ppSrc, int& bitPos, int numBitsLUT, int& value) const
  {
    if (!ppSrc || !(*ppSrc) || bitPos < 0 || bitPos >= 32)
      return false;

    // first get the next (up to) 12 bits as a copy
    int valTmp = ((**ppSrc) << bitPos) >> (32 - numBitsLUT);
    if (32 - bitPos < numBitsLUT)
    {
      valTmp |= (*(*ppSrc + 1)) >> (64 - bitPos - numBitsLUT);
    }

    if (m_decodeLUT[valTmp].first >= 0)    // if there, move the correct number of bits and done
    {
      value = m_decodeLUT[valTmp].second;
      bitPos += m_decodeLUT[valTmp].first;
      if (bitPos >= 32)
      {
        bitPos -= 32;
        (*ppSrc)++;
      }
      return true;
    }

    // if not there, go through the tree (slower)

    if (!m_root)
      return false;

    // skip leading 0 bits before entering the tree
    bitPos += m_numBitsToSkipInTree;
    if (bitPos >= 32)
    {
      bitPos -= 32;
      (*ppSrc)++;
    }

    const Node* node = m_root;
    value = -1;
    while (value < 0)
    {
      int bit = ((**ppSrc) << bitPos) >> 31;
      bitPos++;
      if (bitPos == 32)
      {
        bitPos = 0;
        (*ppSrc)++;
      }

      node = bit ? node->child1 : node->child0;

      if (node->value >= 0)    // reached a leaf node
        value = node->value;
    }

    return true;
  }

}    // namespace

#endif
