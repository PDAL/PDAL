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

#include <boost/test/unit_test.hpp>

#include <pdal/plang/AstNode.hpp>
#include <pdal/plang/SymbolTable.hpp>
#include <pdal/plang/Program.hpp>


BOOST_AUTO_TEST_SUITE(PLangInternalTest)


using pdal::plang::variant_t;
using pdal::plang::Parser;


BOOST_AUTO_TEST_CASE(PLangInternalTest_ast)
{
    using namespace pdal::plang;

    AstNode* a1 = new AstBinaryOp(new AstConstant(1.1), new AstNegate(new AstBinaryOp(new AstConstant(2.2), new AstConstant(2.0), NodeType_Multiply)), NodeType_Add);
    AstNode* a2 = new AstBinaryOp(new AstConstant(1.1), new AstNegate(new AstBinaryOp(new AstConstant(2.2), new AstConstant(2.0), NodeType_Multiply)), NodeType_Add);
    assert(*a1 == *a2);
   
    std::vector<Symbol*> syms;
    SymbolTable symtab(syms);
    AstNode* b = a1->simplify(symtab);
    AstNode* c = new AstConstant(1.1 + -(2.2*2.0));

    assert(*b == *c);

    delete a2;
    delete b;
    delete c;

    return;
}


BOOST_AUTO_TEST_SUITE_END()
