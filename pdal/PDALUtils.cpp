/******************************************************************************
* Copyright (c) 2014, Michael P. Gerlek (mpg@flaxen.com)
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

#include <pdal/PDALUtils.hpp>

#include <arbiter/arbiter.hpp>

#include <pdal/KDIndex.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace std;

namespace pdal
{

namespace {

void toJSON(const MetadataNode& m, std::ostream& o, int level);
void arrayToJSON(const MetadataNodeList& children, std::ostream& o, int level);
void arrayEltToJSON(const MetadataNode& m, std::ostream& o, int level);
void subnodesToJSON(const MetadataNode& parent, std::ostream& o, int level)
{
    const std::string indent(level * 2, ' ');

    std::vector<std::string> names = parent.childNames();

    o << indent << "{" << endl;
    for (auto ni = names.begin(); ni != names.end(); ++ni)
    {
        MetadataNodeList children = parent.children(*ni);

        MetadataNode& node = *children.begin();
        if (node.kind() == MetadataType::Array)
        {
            o << indent << "  \"" << node.name() << "\":" << std::endl;
            arrayToJSON(children, o, level + 1);
        }
        else
            toJSON(node, o, level + 1);
        if (ni != names.rbegin().base() - 1)
            o << ",";
        o << std::endl;
    }
    o << indent << "}";
}

void arrayToJSON(const MetadataNodeList& children, std::ostream& o, int level)
{
    const std::string indent(level * 2, ' ');

    o << indent << "[" << std::endl;
    for (auto ci = children.begin(); ci != children.end(); ++ci)
    {
        const MetadataNode& m = *ci;

        arrayEltToJSON(m, o, level + 1);
        if (ci != children.rbegin().base() - 1)
            o << ",";
        o << std::endl;
    }
    o << indent << "]";
}

void arrayEltToJSON(const MetadataNode& m, std::ostream& o, int level)
{
    std::string indent(level * 2, ' ');
    std::string value = m.jsonValue();
    bool children = m.hasChildren();

    // This is a case from XML.  In JSON, you can't have two values.
    if (!value.empty() && children)
    {
        o << value << "," << std::endl;
        subnodesToJSON(m, o, level);
    }
    else if (!value.empty())
        o << indent << value;
    else
        subnodesToJSON(m, o, level);
    // There is the case where we have a name and no value to handle.  What
    // should be done?
}

void toJSON(const MetadataNode& m, std::ostream& o, int level)
{
    std::string indent(level * 2, ' ');
    std::string name = m.name();
    std::string value = m.jsonValue();
    bool children = m.hasChildren();

    if (name.empty())
        name = "unnamed";

    // This is a case from XML.  In JSON, you can't have two values.
    if (!value.empty() && children)
    {
        o << indent << "\"" << name << "\": " << value << "," << std::endl;
        o << indent << "\"" << name << "\": ";
        subnodesToJSON(m, o, level);
    }
    else if (!value.empty())
        o << indent << "\"" << name << "\": " << value;
    else
    {
        o << indent << "\"" << name << "\":" << std::endl;
        subnodesToJSON(m, o, level);
    }
    // There is the case where we have a name and no value to handle.  What
    // should be done?
}

} // unnamed namespace

namespace Utils
{

std::string toJSON(const MetadataNode& m)
{
    std::ostringstream o;

    toJSON(m, o);
    return o.str();
}

void toJSON(const MetadataNode& m, std::ostream& o)
{
    if (m.name().empty())
        pdal::subnodesToJSON(m, o, 0);
    else if (m.kind() == MetadataType::Array)
        pdal::arrayToJSON(m.children(), o, 0);
    else
    {
        o << "{" << std::endl;
        pdal::toJSON(m, o, 1);
        o << std::endl;
        o << "}";
    }
    o << std::endl;
}

namespace
{

std::string tempFilename(const std::string& path)
{
    const std::string tempdir(arbiter::getTempPath());
    const std::string basename(arbiter::getBasename(path));

    return arbiter::join(tempdir, basename);
}

// RAII handling of a temp file to make sure file gets deleted.
class TempFile
{
public:
    TempFile(const std::string path) : m_filename(path)
    {}

    virtual ~TempFile()
        { FileUtils::deleteFile(m_filename); }

    const std::string& filename()
        { return m_filename; }

private:
    std::string m_filename;
};

class ArbiterOutStream : public std::ofstream
{
public:
    ArbiterOutStream(const std::string& localPath,
            const std::string& remotePath, std::ios::openmode mode) :
        std::ofstream(localPath, mode), m_remotePath(remotePath),
        m_localFile(localPath)
    {}

    virtual ~ArbiterOutStream()
    {
        close();
        arbiter::Arbiter a;
        a.put(m_remotePath, a.getBinary(m_localFile.filename()));
    }

    std::string m_remotePath;
    TempFile m_localFile;
};

class ArbiterInStream : public std::ifstream
{
public:
    ArbiterInStream(const std::string& localPath, const std::string& remotePath,
            std::ios::openmode mode) :
        m_localFile(localPath)
    {
        arbiter::Arbiter a;
        a.put(localPath, a.getBinary(remotePath));
        open(localPath, mode);
    }

    TempFile m_localFile;
};

}  // unnamed namespace

/**
  Create a file (may be on a supported remote filesystem).

  \param path  Path to file to create.
  \param asBinary  Whether the file should be written in binary mode.
  \return  Pointer to the created stream, or NULL.
*/
std::ostream *createFile(const std::string& path, bool asBinary)
{
    ostream *ofs(nullptr);

    if (isRemote(path))
    {
        arbiter::Arbiter a;
        if (!a.hasDriver(path))
            return ofs;
        try
        {
            ofs = new ArbiterOutStream(tempFilename(path), path,
                asBinary ? ios::out | ios::binary : ios::out);
        }
        catch (arbiter::ArbiterError&)
        {}
        if (ofs && !ofs->good())
        {
            delete ofs;
            ofs = nullptr;
        }
    }
    else
        ofs = FileUtils::createFile(path, asBinary);
    return ofs;
}


/**
  Open a file (potentially on a remote filesystem).

  \param path  Path (potentially remote) of file to open.
  \param asBinary  Whether the file should be opened binary.
  \return  Pointer to stream opened for input.
*/

bool isRemote(const std::string& path)
{
    const StringList prefixes
        { "s3://", "gs://", "dropbox://", "http://", "https://" };

    for (const string& prefix : prefixes)
        if (Utils::startsWith(path, prefix))
            return true;
    return false;
}


std::string fetchRemote(const std::string& path)
{
    std::string temp = tempFilename(path);
    arbiter::Arbiter a;
    a.put(temp, a.getBinary(path));
    return temp;
}

std::istream *openFile(const std::string& path, bool asBinary)
{
    if (isRemote(path))
    {
        arbiter::Arbiter a;
        if (!a.hasDriver(path))
            return nullptr;
        try
        {
            return new ArbiterInStream(tempFilename(path), path,
                asBinary ? ios::in | ios::binary : ios::in);
        }
        catch (arbiter::ArbiterError&)
        {
            return nullptr;
        }
    }
    return FileUtils::openFile(path, asBinary);
}

/**
  Close an output stream.

  \param out  Stream to close.
*/
void closeFile(std::ostream *out)
{
    FileUtils::closeFile(out);
}


/**
  Close an input stream.

  \param out  Stream to close.
*/
void closeFile(std::istream *in)
{
    FileUtils::closeFile(in);
}


/**
  Check to see if a file exists.

  \param path  Path to file.
  \return  Whether the file exists or not.
*/
bool fileExists(const std::string& path)
{
    if (isRemote(path))
    {
        arbiter::Arbiter a;
        return (a.hasDriver(path) && a.exists(path));
    }

    // Arbiter doesn't handle our STDIN hacks.
    return FileUtils::fileExists(path);
}

double computeHausdorff(PointViewPtr srcView, PointViewPtr candView)
{
    using namespace Dimension;

    KD3Index srcIndex(*srcView);
    srcIndex.build();

    KD3Index candIndex(*candView);
    candIndex.build();

    double maxDistSrcToCand = std::numeric_limits<double>::lowest();
    double maxDistCandToSrc = std::numeric_limits<double>::lowest();

    for (PointId i = 0; i < srcView->size(); ++i)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        PointRef srcPoint = srcView->point(i);
        candIndex.knnSearch(srcPoint, 1, &indices, &sqr_dists);

        if (sqr_dists[0] > maxDistSrcToCand)
            maxDistSrcToCand = sqr_dists[0];
    }

    for (PointId i = 0; i < candView->size(); ++i)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        PointRef candPoint = candView->point(i);
        srcIndex.knnSearch(candPoint, 1, &indices, &sqr_dists);

        if (sqr_dists[0] > maxDistCandToSrc)
            maxDistCandToSrc = sqr_dists[0];
    }

    maxDistSrcToCand = std::sqrt(maxDistSrcToCand);
    maxDistCandToSrc = std::sqrt(maxDistCandToSrc);

    return (std::max)(maxDistSrcToCand, maxDistCandToSrc);
}

} // namespace Utils
} // namespace pdal
