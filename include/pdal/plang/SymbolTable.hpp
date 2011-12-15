/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#ifndef SYMBOLTABLE_H
#define SYMBOLTABLE_H

#include <pdal/pdal_internal.hpp>

#include <pdal/plang/Parser.hpp>

#include <vector>
#include <iostream>


namespace pdal { namespace plang {


enum DataType
{
    DataType_Unknown,
    DataType_Uint8, DataType_Int8, 
    DataType_Uint16, DataType_Int16, 
    DataType_Uint32, DataType_Int32, 
    DataType_Uint64, DataType_Int64, 
    DataType_Float32, DataType_Float64, 
    DataType_Bool
};


class PDAL_DLL SymbolName
{
public:
    SymbolName(const std::string&);
    SymbolName(char, std::vector<char>); // special hacky ctor used by the parser
    ~SymbolName();

    const std::string& getName() const { return m_name; }

private:
    std::string m_name;
};


class PDAL_DLL Symbol
{
public:
    Symbol(const SymbolName*, DataType);
    ~Symbol();

    virtual void print() const;

    const std::string& getName() const { return m_name; }
    DataType getDataType() const { return m_datatype; }

    void isFree(bool b) { m_isFree = b; }
    bool isFree() const { return m_isFree; }

    void isDefined(bool b) { m_isDefined = b; }
    bool isDefined() const { return m_isDefined; }

    void isUsed(bool b) { m_isUsed = b; }
    bool isUsed() const { return m_isUsed; }

    bool hasValue() const { return m_hasValue; }

    variant_t getVariantValue() const
    {
        assert(m_hasValue);
        return m_value;
    }

    template<typename T>
    T getValue() const
    {
        assert(m_hasValue);
        return boost::get<T>(m_value);
    }

    void setValue(variant_t value)
    {
        m_hasValue = true;
        m_value = value;
    }

private:
    std::string m_name;
    DataType m_datatype;
    
    bool m_isFree;      // true iff has an upwards exposed use
    bool m_isDefined;
    bool m_isUsed;

    bool m_hasValue;
    variant_t m_value;
};


class PDAL_DLL SymbolTable
{
public:
    SymbolTable(std::vector<Symbol*>&);
    ~SymbolTable();

    // takes ownership
    void add(Symbol* symbol);

    Symbol* get(const std::string) const;
    bool has(const std::string) const;

    void print() const;

private:
    std::vector<Symbol*> m_symbols;
};


} } // namespaces


#endif
