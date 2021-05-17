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

#ifndef _WIN32
#include <dlfcn.h>
#endif

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

uintmax_t fileSize(const std::string& path)
{
    uintmax_t size = 0;
    if (isRemote(path))
    {
        std::unique_ptr<std::size_t> pSize = arbiter::Arbiter().tryGetSize(path);
        if (pSize)
            size = *pSize;
    }
    else
        size = FileUtils::fileSize(path);
    return size;
}

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

    KD3Index &srcIndex = srcView->build3dIndex();
    KD3Index &candIndex = candView->build3dIndex();

    double maxDistSrcToCand = std::numeric_limits<double>::lowest();
    double maxDistCandToSrc = std::numeric_limits<double>::lowest();

    for (PointRef p : *srcView)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        candIndex.knnSearch(p, 1, &indices, &sqr_dists);

        if (sqr_dists[0] > maxDistSrcToCand)
            maxDistSrcToCand = sqr_dists[0];
    }

    for (PointRef q : *candView)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        srcIndex.knnSearch(q, 1, &indices, &sqr_dists);

        if (sqr_dists[0] > maxDistCandToSrc)
            maxDistCandToSrc = sqr_dists[0];
    }

    maxDistSrcToCand = std::sqrt(maxDistSrcToCand);
    maxDistCandToSrc = std::sqrt(maxDistCandToSrc);

    return (std::max)(maxDistSrcToCand, maxDistCandToSrc);
}

std::pair<double, double> computeHausdorffPair(PointViewPtr viewA,
                                               PointViewPtr viewB)
{
    // Computes both the max and mean of all nearest neighbor distances from
    // each point in the PointView to those in the KD3Index.
    auto compute = [](PointViewPtr view, KD3Index& index) {
        double max_distance = std::numeric_limits<double>::lowest();
        double M1(0.0);
        for (PointRef p : *view)
        {
            PointIdList indices(1);
            std::vector<double> sqr_dists(1);
            index.knnSearch(p, 1, &indices, &sqr_dists);

            if (sqr_dists[0] > max_distance)
                max_distance = sqr_dists[0];

            double delta = std::sqrt(sqr_dists[0]) - M1;
            double delta_n = delta / (p.pointId() + 1);
            M1 += delta_n;
        }
        max_distance = std::sqrt(max_distance);
        return std::pair<double, double>{max_distance, M1};
    };

    // First, test from view A to view B...
    KD3Index& indexB = viewB->build3dIndex();
    std::pair<double, double> a2b = compute(viewA, indexB);

    // then recompute from view B to view A.
    KD3Index& indexA = viewA->build3dIndex();
    std::pair<double, double> b2a = compute(viewB, indexA);

    // The original Hausdorff metric is the max of the max distances from A to B
    // and vice versa.
    double original = (std::max)(a2b.first, b2a.first);

    // The modified Hausdorff metric is the max of the mean distances from A to
    // B and vice versa.
    double modified = (std::max)(a2b.second, b2a.second);

    // Return both the original and modified metrics.
    return std::pair<double, double>{original, modified};
}

std::string dllDir()
{
    std::string s;

#ifdef _WIN32
    HMODULE hm = NULL;

    if (GetModuleHandleEx(
        GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
        GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
        (LPCSTR)&dllDir, &hm))
    {
        char path[MAX_PATH];
        DWORD cnt = GetModuleFileNameA(hm, path, sizeof(path));
        if (cnt > 0 && cnt < MAX_PATH)
            s = path;
    }
#else
    Dl_info info;
    if (dladdr((const void *)dllDir, &info))
        s = info.dli_fname;
#endif
    return FileUtils::getDirectory(s);
}


double computeChamfer(PointViewPtr srcView, PointViewPtr candView)
{
    using namespace Dimension;

    KD3Index &srcIndex = srcView->build3dIndex();
    KD3Index &candIndex = candView->build3dIndex();

    double sum1(0.0);
    for (PointRef p : *srcView)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        candIndex.knnSearch(p, 1, &indices, &sqr_dists);
        sum1 += sqr_dists[0];
    }

    double sum2(0.0);
    for (PointRef q : *candView)
    {
        PointIdList indices(1);
        std::vector<double> sqr_dists(1);
        srcIndex.knnSearch(q, 1, &indices, &sqr_dists);
        sum2 += sqr_dists[0];
    }

    return sum1 + sum2;
}

} // namespace Utils
} // namespace pdal
