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

#include <pdal/plang/SymbolTable.hpp>

#include <pdal/plang/AstUtils.hpp>

#include <string>
#include <iostream>


namespace pdal { namespace plang {

//---------------------------------------------------------------------------


SymbolName::SymbolName(const std::string& name)
    : m_name(name)
{
    return;
}


SymbolName::SymbolName(char c, std::vector<char> cc)
    : m_name("")
{
    m_name.push_back(c);    
    for (unsigned int i=0; i<cc.size(); i++)
        m_name.push_back(cc[i]);
    return;
}


//---------------------------------------------------------------------------


Symbol::Symbol(const SymbolName* name, DataType datatype)
    : m_name(name->getName())
    , m_datatype(datatype)
    , m_hasValue(false)
    , m_isFree(false)
    , m_isUsed(false)
    , m_isDefined(false)
{
    return;
}


Symbol::~Symbol()
{
    return;
}


void Symbol::print() const
{
    std::cout << "Symbol " << m_name << " " << AstUtils::getName(m_datatype) << "\n";
}


//---------------------------------------------------------------------------


SymbolTable::SymbolTable(std::vector<Symbol*>& syms)
    : m_symbols(syms)
{
    return;
}


SymbolTable::~SymbolTable()
{
    for (unsigned int i=0; i<m_symbols.size(); i++)
    {
        delete m_symbols[i];
    }
    return;
}


Symbol* SymbolTable::get(const std::string name) const
{
    for (unsigned int i=0; i<m_symbols.size(); i++)
    {
        Symbol* sym = m_symbols[i];
        if (sym->getName() == name)
        {
            return sym;
        }
    }

    throw;
}


bool SymbolTable::has(const std::string name) const
{
    for (unsigned int i=0; i<m_symbols.size(); i++)
    {
        const Symbol* sym = m_symbols[i];
        if (sym->getName() == name)
        {
            return true;
        }
    }

    return false;
}


void SymbolTable::add(Symbol* symbol)
{
    m_symbols.push_back(symbol);
}


void SymbolTable::print() const
{
    for (unsigned int i=0; i<m_symbols.size(); i++)
    {
        const Symbol* sym = m_symbols[i];
        sym->print();
    }

    return;
}

} } // namespaces

