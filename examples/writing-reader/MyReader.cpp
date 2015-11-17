// MyReader.cpp

#include "MyReader.hpp"

namespace pdal
{
  static PluginInfo const s_info = PluginInfo(
    "readers.myreader",
    "My Awesome Reader",
    "http://link/to/documentation" );

  CREATE_SHARED_PLUGIN(1, 0, MyReader, Reader, s_info)

  std::string MyReader::getName() const { return s_info.name; }

  Options MyReader::getDefaultOptions()
  {
    Options options;
    Option z_scale("z_scale", 1.0);
    options.add(z_scale);
    return options;
  }

  void MyReader::processOptions(const Options& options)
  {
    m_scale_z = options.getValueOrDefault<double>("z_scale", 1.0);
  }

  void MyReader::addDimensions(PointLayoutPtr layout)
  {
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    layout->registerOrAssignDim("MyData", Dimension::Type::Unsigned64);
  }

  Dimension::IdList MyReader::getDefaultDimensions()
  {
    Dimension::IdList ids;

    ids.push_back(Dimension::Id::X);
    ids.push_back(Dimension::Id::Y);
    ids.push_back(Dimension::Id::Z);

    return ids;
  }

  void MyReader::ready(PointTableRef)
  {
    SpatialReference ref("EPSG:4385");
    setSpatialReference(ref);
  }


  template <typename T>
  T convert(const StringList& s, const std::string& name, size_t fieldno)
  {
      T output;
      bool bConverted = Utils::fromString(s[fieldno], output);
      if (!bConverted)
      {
          std::stringstream oss;
          oss << "Unable to convert " << name << ", " << s[fieldno] << ", to double";
          throw pdal_error(oss.str());
      }

      return output;
  }


  point_count_t MyReader::read(PointViewPtr view, point_count_t count)
  {
    PointLayoutPtr layout = view->layout();
    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;

    m_stream.reset(new ILeStream(m_filename));

    size_t HEADERSIZE(1);
    size_t skip_lines(std::max(HEADERSIZE, (size_t)m_index));
    size_t line_no(1);
    for (std::string line; std::getline(*m_stream->stream(), line); line_no++)
    {
      if (line_no <= skip_lines)
      {
        continue;
      }

      // MyReader format:  X::Y::Z::Data
      StringList s = Utils::split2(line, ':');

      unsigned long u64(0);
      if (s.size() != 4)
      {
        std::stringstream oss;
        oss << "Unable to split proper number of fields.  Expected 4, got " << s.size();
        throw pdal_error(oss.str());
      }

      std::string name("X");
      view->setField(Dimension::Id::X, nextId, convert<double>(s, name, 0));

      name = "Y";
      view->setField(Dimension::Id::Y, nextId, convert<double>(s, name, 1));

      name = "Z";
      double z = convert<double>(s, name, 2) * m_scale_z;
      view->setField(Dimension::Id::Z, nextId, z);

      name = "MyData";
      view->setField(layout->findProprietaryDim(name), nextId, convert<unsigned int>(s, name, 3));

      nextId++;
      if (m_cb)
        m_cb(*view, nextId);
    }
    m_index = nextId;
    numRead = nextId;

    return numRead;
  }

  void MyReader::done(PointTableRef)
  {
    m_stream.reset();
  }

} //namespace pdal
