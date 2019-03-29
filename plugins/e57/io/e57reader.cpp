#include <iostream>
#include <E57Exception.h>

#include "e57reader.hpp"
#include "utils.hpp"

namespace pdal
{

E57Reader::ChunkReader::ChunkReader(pdal::point_count_t pointOffset,pdal::point_count_t maxPointRead,
	std::shared_ptr<e57::Scan> scan, 
    const std::set<std::string>& e57Dimensions):
    m_startIndex(0), m_defaultChunkSize(1e6), m_maxPointRead(maxPointRead), m_scan(scan)
{
    // Initialise the read buffers
    for (auto& dimension: e57Dimensions)
    {
        m_doubleBuffers[dimension] = std::vector<double>(m_defaultChunkSize,0);
    }
	auto vectorNode = scan->getPoints();
    auto prototype = static_cast<e57::StructureNode>(vectorNode.prototype());
    for (auto& keyValue: m_doubleBuffers)
    {
        m_e57buffers.emplace_back(vectorNode.destImageFile(), keyValue.first,
             keyValue.second.data(), m_defaultChunkSize, true, (prototype.get(keyValue.first).type() == e57::E57_SCALED_INTEGER) );
    }

	m_pointOffset = pointOffset;

    // Setup the reader
    m_dataReader = std::unique_ptr<e57::CompressedVectorReader>(
        new e57::CompressedVectorReader(vectorNode.reader(m_e57buffers)));
}

E57Reader::ChunkReader::~ChunkReader()
{
	if (m_dataReader)
	{
		m_dataReader->close();
	}
}

bool E57Reader::ChunkReader::isInScope(pdal::point_count_t index) const
{
	pdal::point_count_t actualIndex = index - m_pointOffset;
	return (actualIndex>=0 && actualIndex < m_maxPointRead);
}

bool E57Reader::ChunkReader::isInChunk(pdal::point_count_t index) const
{
	pdal::point_count_t actualIndex = index - m_pointOffset;
    if (actualIndex < m_startIndex)
    {
       throw std::out_of_range("E57 reader tries to go back in the data. Not allowed.");
    }
    return actualIndex < (m_startIndex+m_defaultChunkSize);
}

void E57Reader::ChunkReader::setPoint(pdal::point_count_t pointIndex, pdal::PointRef point, 
     const std::set<std::string>& e57Dimensions) const
{
	pdal::point_count_t actualIndex = pointIndex - m_pointOffset;
    pdal::point_count_t index = actualIndex - m_startIndex;

	// Sets values from buffer arrays for each dimension in the scan
    for (auto& keyValue: m_doubleBuffers)
    {
		auto pdalDimension = pdal::e57plugin::e57ToPdal(keyValue.first);
		if (pdalDimension != pdal::Dimension::Id::Unknown)
        {
			double value;
			// Rescale the value if needed so that if fits Pdal expectations
			try 
			{
				auto minmax = m_scan->getLimits(pdalDimension);
				value = pdal::e57plugin::rescaleE57ToPdalValue(keyValue.first,keyValue.second[index],minmax);
			}
			catch (std::out_of_range e)
			{
				value = keyValue.second[index];
			}			
			point.setField(pdalDimension,value);
		}
    }

	// Applies pose transformation if needed
	if (m_scan->hasPose())
	{
		m_scan->transformPoint(point);
	}
}

pdal::point_count_t E57Reader::ChunkReader::read(pdal::point_count_t  index)
{
	pdal::point_count_t actualIndex = index - m_pointOffset;
    m_startIndex = actualIndex;
    return m_dataReader->read();
}



static PluginInfo const s_info
{
	"readers.e57",
	"E57 Reader",
	"http://libe57.org/"
};

CREATE_SHARED_STAGE(E57Reader, s_info)

E57Reader::~E57Reader()
{
	if (m_imf)
	{
		m_imf->close();
	}
}

E57Reader::E57Reader(std::string filename) : Reader(), Streamable()
{
	m_filenameManual = filename;
	initialize();
}

std::string E57Reader::getName() const
{
	return s_info.name;
}

void E57Reader::initialize() 
{
	if (m_filename.size() == 0)
	{
		m_filename = m_filenameManual;
	}
	openFile_(m_filename);
	extractScans_();
	m_pointCount = getNumberPoints_();
	m_currentScanIndex = -1;
	m_chunk = std::unique_ptr<ChunkReader>(nullptr);
	m_currentPoint = 0;	
}

void E57Reader::addDimensions(PointLayoutPtr layout)
{
	std::set<pdal::Dimension::Id> supportedDimensions;
	if (m_scans.size() == 0)
	{
		return;
	}
	
	auto commonDimensions = getDimensions();
	for (auto& dim: commonDimensions)
	{
		auto pdalDimension = pdal::e57plugin::e57ToPdal(dim);
		if (pdalDimension != pdal::Dimension::Id::Unknown)
		{
			layout->registerDim(pdalDimension);
		}
	}
}

point_count_t E57Reader::read(PointViewPtr view, point_count_t count)
{
	// How many do we read
	PointId nextId = view->size();
	point_count_t remainingInput = m_pointCount - nextId;
	point_count_t toReadCount = std::min(count, remainingInput);

	// m_scans[0]->read(nextId,toReadCount,view);

	point_count_t remaining = toReadCount;
    while (remaining--)
    {
        PointRef point(view->point(nextId));
		m_currentPoint = nextId;
        processOne(point);
        nextId++;
    }

	// Return what was read
    return toReadCount;
}

bool E57Reader::processOne(pdal::PointRef& point)
{
	// Did we go too far?
	if (m_currentPoint >= m_pointCount)
    {
        return false;
    }
	// Are we not in the same scan?
	if (!m_chunk || !m_chunk->isInScope(m_currentPoint))
	{
		// setup new reader
		setupReader_(m_currentPoint);
		m_chunk->read(m_currentPoint);
	}

    if (!m_chunk->isInChunk(m_currentPoint))
    {
        m_chunk->read(m_currentPoint);
    }
    m_chunk->setPoint(m_currentPoint,point, getDimensions());
	m_currentPoint++;
	return true;
}

std::string E57Reader::getHeader() const
{
	e57::StructureNode root = m_imf->root();
	std::string info = "";
	unsigned childCount = root.childCount();
	for (unsigned i =0; i< childCount; i++) 
	{
		e57::Node child = root.get(i);
		info += pdal::e57plugin::getDescription(child);
		info += '\n';
	}
	return info;
}

std::string E57Reader::getSummary() const 
{
	std::string info = getHeader();
	info += "This file contains " + std::to_string(m_scans.size()) + " scans.\n";
	
	for (unsigned i = 0; i < m_scans.size(); i++)    
	{
		info += "    Scan " + std::to_string(i+1) + " with " + std::to_string(m_scans[i]->getNumPoints()) +" points.\n";
	}
	return info;
}

std::vector<std::shared_ptr<e57::Scan>> E57Reader::getScans() const
{
	return m_scans;
}

int E57Reader::getScanIndex(pdal::point_count_t pointIndex) const
{
	if (pointIndex > m_pointCount)
	{
		return -1;
	}
	pdal::point_count_t counter = 0;
	for (auto i=0; i < m_scans.size(); i++)
	{
		counter += m_scans[i]->getNumPoints();
		if (pointIndex <counter)
		{
			return i;
		}
	}
	return -1;
}

std::set<std::string> E57Reader::getDimensions()
{
	if (!m_validDimensions.empty())
	{
		return m_validDimensions;
	}

	// Extract smallest common denominator accross all scans (Should we do union of dimensions instead?)
	auto commonDimensions = m_scans[0]->getDimensions();
	std::vector<std::string> dimensionsToRemove;
	for (auto scan: m_scans)
	{
		auto newDims = scan->getDimensions();
		for (auto dim: commonDimensions)
		{
			if (newDims.find(dim) == newDims.end())
			{
				dimensionsToRemove.push_back(dim);
			}
		}
	}
	for (auto dim: dimensionsToRemove)
	{
		commonDimensions.erase(dim);
	}
	m_validDimensions = commonDimensions;
	return m_validDimensions;
}

pdal::point_count_t E57Reader::getNumberPoints() const 
{
	return m_pointCount;
}

void E57Reader::openFile_(std::string filename)
{
	try
	{	
		m_imf = std::unique_ptr<e57::ImageFile>(new e57::ImageFile(filename,"r",e57::CHECKSUM_POLICY_SPARSE));	
		if (!m_imf->isOpen())
		{
			std::cout<<"Failed opening the file"<<std::endl;
		}

		static const e57::ustring normalsExtension("http://www.libe57.org/E57_NOR_surface_normals.txt");
		e57::ustring _normalsExtension;
		if (!m_imf->extensionsLookupPrefix("nor", _normalsExtension)) //the extension may already be registered
		{
			m_imf->extensionsAdd("nor", normalsExtension);
		}		
	}
	catch(const e57::E57Exception& e)
	{
        std::cout<< "E57 error with code ";
		throw pdal_error(e.context());
	}
	catch(...)
	{
		std::string msg("Unknown error");
		std::cout<< msg << std::endl;
		throw pdal_error(msg);
	}
}

void E57Reader::setupReader_(pdal::point_count_t pointNumber)
{
	int currentScan = getScanIndex(pointNumber);
	if (currentScan == -1)
	{
		throw std::out_of_range("Something went wrong in processing a point");
	}
	pdal::point_count_t offset = 0;
	pdal::point_count_t maxRead = m_scans[0]->getNumPoints();
	for (int i=1; i<=currentScan; i++)
	{
		maxRead = m_scans[i]->getNumPoints();
		offset += m_scans[i-1]->getNumPoints();
	}

	m_chunk.reset();
	auto pointNode = m_scans[currentScan]->getPoints();
	m_chunk = std::unique_ptr<ChunkReader>(new ChunkReader(offset,maxRead,m_scans[currentScan],getDimensions()));
}

pdal::point_count_t E57Reader::getNumberPoints_() const 
{
	pdal::point_count_t count = 0;
	for (auto& scan: m_scans)    
	{
		count += scan->getNumPoints();
	}
	return count;
}

void E57Reader::extractScans_()
{
	e57::StructureNode root = m_imf->root();
	if (!root.isDefined("/data3D"))
	{
		throw std::invalid_argument("File does not contain valid 3D data");
	}
	e57::VectorNode n(root.get("/data3D")); //E57 standard: "data3D is a vector for storing an arbitrary number of 3D data sets "
	for (unsigned i = 0; i < n.childCount(); i++)
	{
		try {
			e57::StructureNode scanNode(n.get(i));
			m_scans.emplace_back(new e57::Scan(scanNode));
		}
		catch (const e57::E57Exception& e)
		{
			std::cout<<e.what()<<std::endl;
		}
		catch (const std::exception& e)
		{
			std::cout<<e.what()<<std::endl;
		}
	}
}
}