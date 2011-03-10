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

#ifndef INCLUDED_APPLICATION_HPP
#define INCLUDED_APPLICATION_HPP

#include <iosfwd>


#ifdef _MSC_VER
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#endif
#include <boost/program_options.hpp>
#ifdef _MSC_VER
#  pragma warning(pop)
#endif


//
// The pplication base class gives us these common options:
//    --help / -h
//    --verbose / -v
//    --version
//    --timer
//
class Application
{
public:
    // call this, to start the machine
    int run();

protected:
    // implement this, with calls to addOptionSet()
    virtual void addOptions() = 0;

    // implement this, to do sanity checking of cmd line
    // return false if the user gave us bad options
    virtual bool validateOptions() { return true; }

    // implement this, to do your actual work
    virtual int execute() = 0;

protected:
    Application(int argc, char* argv[], const std::string& appName);
    void addOptionSet(boost::program_options::options_description* options);
    void addPositionalOption(const char* name, int max_count);
    bool isVerbose() const;
    bool hasOption(const std::string& name);
    void usageError(const std::string&);
    void runtimeError(const std::string&);

private:
    void parseOptions();
    void outputHelp();
    void outputVersion();
    void addBasicOptionSet();

    bool m_isVerbose;
    const int m_argc;
    char** m_argv;
    const std::string m_appName;

    std::vector<boost::program_options::options_description*> m_options;
    boost::program_options::positional_options_description m_positionalOptions;
    boost::program_options::variables_map m_variablesMap;

    Application& operator=(const Application&); // not implemented
    Application(const Application&); // not implemented
};

#endif
