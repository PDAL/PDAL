#include <Python.h>

#include <boost/uuid/string_generator.hpp>

//#include <pdal/plang/Plang.hpp>
#include <pdal/Metadata.hpp>

namespace pdal
{
namespace plang
{

PyObject *fromMetadata(MetadataNode m)
{
    std::string name = m.name();
    std::string value = m.value();
    std::string type = m.type();
    std::string description = m.description();

    MetadataNodeList children = m.children();
    PyObject *submeta = NULL;
    if (children.size())
    {
        submeta = PyList_New(0);
        for (MetadataNode& child : children)
            PyList_Append(submeta, fromMetadata(child));
    }
    PyObject *data = PyTuple_New(5);
    PyTuple_SetItem(data, 0, PyString_FromString(name.data()));
    PyTuple_SetItem(data, 1, PyString_FromString(value.data()));
    PyTuple_SetItem(data, 2, PyString_FromString(type.data()));
    PyTuple_SetItem(data, 3, PyString_FromString(description.data()));
    PyTuple_SetItem(data, 4, submeta);

    return data;
}

void addMetadata(PyObject *list, MetadataNode m)
{
    if (!PyList_Check(list))
        return;
    for (Py_ssize_t i = 0; i < PyList_Size(list); ++i)
    {
        PyObject *tuple = PyList_GetItem(list, i);
        if (!PyTuple_Check(tuple) || PyTuple_Size(tuple) != 5)
            continue;
        PyObject *o;
        o = PyTuple_GetItem(tuple, 0);
        if (!o || !PyString_Check(o))
            continue;
        std::string name = PyString_AsString(o);

        o = PyTuple_GetItem(tuple, 1);
        if (!o || !PyString_Check(o))
            continue;
        std::string value = PyString_AsString(o);

        o = PyTuple_GetItem(tuple, 2);
        if (!o || !PyString_Check(o))
            continue;
        std::string type = PyString_AsString(o);
        if (type.empty())
            type = Metadata::inferType(value);

        o = PyTuple_GetItem(tuple, 3);
        if (!o || !PyString_Check(o))
            continue;
        std::string description = PyString_AsString(o);

        PyObject *submeta = PyTuple_GetItem(tuple, 4);
        MetadataNode child =  m.add(name, value, type, description);
        if (submeta)
            addMetadata(submeta, child);
    }
}

} // namespace plang
} // namespace pdal
