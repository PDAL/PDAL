/// Arbiter amalgamated header (https://github.com/connormanning/arbiter).
/// It is intended to be used with #include "arbiter.hpp"

// Git SHA: 18a41e535a428de1cb907b65601abd18d1d4b9b6

// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: LICENSE
// //////////////////////////////////////////////////////////////////////

/*
The MIT License (MIT)

Copyright (c) 2015 Connor Manning

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.


*/

// //////////////////////////////////////////////////////////////////////
// End of content of file: LICENSE
// //////////////////////////////////////////////////////////////////////





#pragma once
/// If defined, indicates that the source file is amalgamated
/// to prevent private header inclusion.
#define ARBITER_IS_AMALGAMATION
#define ARBITER_CUSTOM_NAMESPACE pdal

// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/third/xml/rapidxml.hpp
// //////////////////////////////////////////////////////////////////////

#ifndef RAPIDXML_HPP_INCLUDED
#define RAPIDXML_HPP_INCLUDED

/*
========================================================================
Copyright (c) 2006, 2007 Marcin Kalicinski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
========================================================================
*/

// Copyright (C) 2006, 2009 Marcin Kalicinski
// Version 1.13
// Revision $DateTime: 2009/05/13 01:46:17 $
//! \file rapidxml.hpp This file contains rapidxml parser and DOM implementation

// If standard library is disabled, user must provide implementations of required functions and typedefs
#if !defined(RAPIDXML_NO_STDLIB)
    #include <cstdlib>      // For std::size_t
    #include <cassert>      // For assert
    #include <new>          // For placement new
#endif

// On MSVC, disable "conditional expression is constant" warning (level 4).
// This warning is almost impossible to avoid with certain types of templated code
#ifdef _MSC_VER
    #pragma warning(push)
    #pragma warning(disable:4127)   // Conditional expression is constant
#endif

///////////////////////////////////////////////////////////////////////////
// RAPIDXML_PARSE_ERROR

#if defined(RAPIDXML_NO_EXCEPTIONS)

#define RAPIDXML_PARSE_ERROR(what, where) { parse_error_handler(what, where); assert(0); }

namespace rapidxml
{
    //! When exceptions are disabled by defining RAPIDXML_NO_EXCEPTIONS,
    //! this function is called to notify user about the error.
    //! It must be defined by the user.
    //! <br><br>
    //! This function cannot return. If it does, the results are undefined.
    //! <br><br>
    //! A very simple definition might look like that:
    //! <pre>
    //! void %rapidxml::%parse_error_handler(const char *what, void *where)
    //! {
    //!     std::cout << "Parse error: " << what << "\n";
    //!     std::abort();
    //! }
    //! </pre>
    //! \param what Human readable description of the error.
    //! \param where Pointer to character data where error was detected.
    void parse_error_handler(const char *what, void *where);
}

#else

#include <exception>    // For std::exception

#define RAPIDXML_PARSE_ERROR(what, where) throw parse_error(what, where)

namespace rapidxml
{

    //! Parse error exception.
    //! This exception is thrown by the parser when an error occurs.
    //! Use what() function to get human-readable error message.
    //! Use where() function to get a pointer to position within source text where error was detected.
    //! <br><br>
    //! If throwing exceptions by the parser is undesirable,
    //! it can be disabled by defining RAPIDXML_NO_EXCEPTIONS macro before rapidxml.hpp is included.
    //! This will cause the parser to call rapidxml::parse_error_handler() function instead of throwing an exception.
    //! This function must be defined by the user.
    //! <br><br>
    //! This class derives from <code>std::exception</code> class.
    class parse_error: public std::exception
    {

    public:

        //! Constructs parse error
        parse_error(const char *what, void *where)
            : m_what(what)
            , m_where(where)
        {
        }

        //! Gets human readable description of error.
        //! \return Pointer to null terminated description of the error.
        virtual const char *what() const throw()
        {
            return m_what;
        }

        //! Gets pointer to character data where error happened.
        //! Ch should be the same as char type of xml_document that produced the error.
        //! \return Pointer to location within the parsed string where error occured.
        template<class Ch>
        Ch *where() const
        {
            return reinterpret_cast<Ch *>(m_where);
        }

    private:

        const char *m_what;
        void *m_where;

    };
}

#endif

///////////////////////////////////////////////////////////////////////////
// Pool sizes

#ifndef RAPIDXML_STATIC_POOL_SIZE
    // Size of static memory block of memory_pool.
    // Define RAPIDXML_STATIC_POOL_SIZE before including rapidxml.hpp if you want to override the default value.
    // No dynamic memory allocations are performed by memory_pool until static memory is exhausted.
    #define RAPIDXML_STATIC_POOL_SIZE (64 * 1024)
#endif

#ifndef RAPIDXML_DYNAMIC_POOL_SIZE
    // Size of dynamic memory block of memory_pool.
    // Define RAPIDXML_DYNAMIC_POOL_SIZE before including rapidxml.hpp if you want to override the default value.
    // After the static block is exhausted, dynamic blocks with approximately this size are allocated by memory_pool.
    #define RAPIDXML_DYNAMIC_POOL_SIZE (64 * 1024)
#endif

#ifndef RAPIDXML_ALIGNMENT
    // Memory allocation alignment.
    // Define RAPIDXML_ALIGNMENT before including rapidxml.hpp if you want to override the default value, which is the size of pointer.
    // All memory allocations for nodes, attributes and strings will be aligned to this value.
    // This must be a power of 2 and at least 1, otherwise memory_pool will not work.
    #define RAPIDXML_ALIGNMENT sizeof(void *)
#endif

namespace rapidxml
{
    // Forward declarations
    template<class Ch> class xml_node;
    template<class Ch> class xml_attribute;
    template<class Ch> class xml_document;

    //! Enumeration listing all node types produced by the parser.
    //! Use xml_node::type() function to query node type.
    enum node_type
    {
        node_document,      //!< A document node. Name and value are empty.
        node_element,       //!< An element node. Name contains element name. Value contains text of first data node.
        node_data,          //!< A data node. Name is empty. Value contains data text.
        node_cdata,         //!< A CDATA node. Name is empty. Value contains data text.
        node_comment,       //!< A comment node. Name is empty. Value contains comment text.
        node_declaration,   //!< A declaration node. Name and value are empty. Declaration parameters (version, encoding and standalone) are in node attributes.
        node_doctype,       //!< A DOCTYPE node. Name is empty. Value contains DOCTYPE text.
        node_pi             //!< A PI node. Name contains target. Value contains instructions.
    };

    ///////////////////////////////////////////////////////////////////////
    // Parsing flags

    //! Parse flag instructing the parser to not create data nodes.
    //! Text of first data node will still be placed in value of parent element, unless rapidxml::parse_no_element_values flag is also specified.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_no_data_nodes = 0x1;

    //! Parse flag instructing the parser to not use text of first data node as a value of parent element.
    //! Can be combined with other flags by use of | operator.
    //! Note that child data nodes of element node take precendence over its value when printing.
    //! That is, if element has one or more child data nodes <em>and</em> a value, the value will be ignored.
    //! Use rapidxml::parse_no_data_nodes flag to prevent creation of data nodes if you want to manipulate data using values of elements.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_no_element_values = 0x2;

    //! Parse flag instructing the parser to not place zero terminators after strings in the source text.
    //! By default zero terminators are placed, modifying source text.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_no_string_terminators = 0x4;

    //! Parse flag instructing the parser to not translate entities in the source text.
    //! By default entities are translated, modifying source text.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_no_entity_translation = 0x8;

    //! Parse flag instructing the parser to disable UTF-8 handling and assume plain 8 bit characters.
    //! By default, UTF-8 handling is enabled.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_no_utf8 = 0x10;

    //! Parse flag instructing the parser to create XML declaration node.
    //! By default, declaration node is not created.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_declaration_node = 0x20;

    //! Parse flag instructing the parser to create comments nodes.
    //! By default, comment nodes are not created.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_comment_nodes = 0x40;

    //! Parse flag instructing the parser to create DOCTYPE node.
    //! By default, doctype node is not created.
    //! Although W3C specification allows at most one DOCTYPE node, RapidXml will silently accept documents with more than one.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_doctype_node = 0x80;

    //! Parse flag instructing the parser to create PI nodes.
    //! By default, PI nodes are not created.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_pi_nodes = 0x100;

    //! Parse flag instructing the parser to validate closing tag names.
    //! If not set, name inside closing tag is irrelevant to the parser.
    //! By default, closing tags are not validated.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_validate_closing_tags = 0x200;

    //! Parse flag instructing the parser to trim all leading and trailing whitespace of data nodes.
    //! By default, whitespace is not trimmed.
    //! This flag does not cause the parser to modify source text.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_trim_whitespace = 0x400;

    //! Parse flag instructing the parser to condense all whitespace runs of data nodes to a single space character.
    //! Trimming of leading and trailing whitespace of data is controlled by rapidxml::parse_trim_whitespace flag.
    //! By default, whitespace is not normalized.
    //! If this flag is specified, source text will be modified.
    //! Can be combined with other flags by use of | operator.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_normalize_whitespace = 0x800;

    // Compound flags

    //! Parse flags which represent default behaviour of the parser.
    //! This is always equal to 0, so that all other flags can be simply ored together.
    //! Normally there is no need to inconveniently disable flags by anding with their negated (~) values.
    //! This also means that meaning of each flag is a <i>negation</i> of the default setting.
    //! For example, if flag name is rapidxml::parse_no_utf8, it means that utf-8 is <i>enabled</i> by default,
    //! and using the flag will disable it.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_default = 0;

    //! A combination of parse flags that forbids any modifications of the source text.
    //! This also results in faster parsing. However, note that the following will occur:
    //! <ul>
    //! <li>names and values of nodes will not be zero terminated, you have to use xml_base::name_size() and xml_base::value_size() functions to determine where name and value ends</li>
    //! <li>entities will not be translated</li>
    //! <li>whitespace will not be normalized</li>
    //! </ul>
    //! See xml_document::parse() function.
    const int parse_non_destructive = parse_no_string_terminators | parse_no_entity_translation;

    //! A combination of parse flags resulting in fastest possible parsing, without sacrificing important data.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_fastest = parse_non_destructive | parse_no_data_nodes;

    //! A combination of parse flags resulting in largest amount of data being extracted.
    //! This usually results in slowest parsing.
    //! <br><br>
    //! See xml_document::parse() function.
    const int parse_full = parse_declaration_node | parse_comment_nodes | parse_doctype_node | parse_pi_nodes | parse_validate_closing_tags;

    ///////////////////////////////////////////////////////////////////////
    // Internals

    //! \cond internal
    namespace internal
    {

        // Struct that contains lookup tables for the parser
        // It must be a template to allow correct linking (because it has static data members, which are defined in a header file).
        template<int Dummy>
        struct lookup_tables
        {
            static const unsigned char lookup_whitespace[256];              // Whitespace table
            static const unsigned char lookup_node_name[256];               // Node name table
            static const unsigned char lookup_text[256];                    // Text table
            static const unsigned char lookup_text_pure_no_ws[256];         // Text table
            static const unsigned char lookup_text_pure_with_ws[256];       // Text table
            static const unsigned char lookup_attribute_name[256];          // Attribute name table
            static const unsigned char lookup_attribute_data_1[256];        // Attribute data table with single quote
            static const unsigned char lookup_attribute_data_1_pure[256];   // Attribute data table with single quote
            static const unsigned char lookup_attribute_data_2[256];        // Attribute data table with double quotes
            static const unsigned char lookup_attribute_data_2_pure[256];   // Attribute data table with double quotes
            static const unsigned char lookup_digits[256];                  // Digits
            static const unsigned char lookup_upcase[256];                  // To uppercase conversion table for ASCII characters
        };

        // Find length of the string
        template<class Ch>
        inline std::size_t measure(const Ch *p)
        {
            const Ch *tmp = p;
            while (*tmp)
                ++tmp;
            return tmp - p;
        }

        // Compare strings for equality
        template<class Ch>
        inline bool compare(const Ch *p1, std::size_t size1, const Ch *p2, std::size_t size2, bool case_sensitive)
        {
            if (size1 != size2)
                return false;
            if (case_sensitive)
            {
                for (const Ch *end = p1 + size1; p1 < end; ++p1, ++p2)
                    if (*p1 != *p2)
                        return false;
            }
            else
            {
                for (const Ch *end = p1 + size1; p1 < end; ++p1, ++p2)
                    if (lookup_tables<0>::lookup_upcase[static_cast<unsigned char>(*p1)] != lookup_tables<0>::lookup_upcase[static_cast<unsigned char>(*p2)])
                        return false;
            }
            return true;
        }
    }
    //! \endcond

    ///////////////////////////////////////////////////////////////////////
    // Memory pool

    //! This class is used by the parser to create new nodes and attributes, without overheads of dynamic memory allocation.
    //! In most cases, you will not need to use this class directly.
    //! However, if you need to create nodes manually or modify names/values of nodes,
    //! you are encouraged to use memory_pool of relevant xml_document to allocate the memory.
    //! Not only is this faster than allocating them by using <code>new</code> operator,
    //! but also their lifetime will be tied to the lifetime of document,
    //! possibly simplyfing memory management.
    //! <br><br>
    //! Call allocate_node() or allocate_attribute() functions to obtain new nodes or attributes from the pool.
    //! You can also call allocate_string() function to allocate strings.
    //! Such strings can then be used as names or values of nodes without worrying about their lifetime.
    //! Note that there is no <code>free()</code> function -- all allocations are freed at once when clear() function is called,
    //! or when the pool is destroyed.
    //! <br><br>
    //! It is also possible to create a standalone memory_pool, and use it
    //! to allocate nodes, whose lifetime will not be tied to any document.
    //! <br><br>
    //! Pool maintains <code>RAPIDXML_STATIC_POOL_SIZE</code> bytes of statically allocated memory.
    //! Until static memory is exhausted, no dynamic memory allocations are done.
    //! When static memory is exhausted, pool allocates additional blocks of memory of size <code>RAPIDXML_DYNAMIC_POOL_SIZE</code> each,
    //! by using global <code>new[]</code> and <code>delete[]</code> operators.
    //! This behaviour can be changed by setting custom allocation routines.
    //! Use set_allocator() function to set them.
    //! <br><br>
    //! Allocations for nodes, attributes and strings are aligned at <code>RAPIDXML_ALIGNMENT</code> bytes.
    //! This value defaults to the size of pointer on target architecture.
    //! <br><br>
    //! To obtain absolutely top performance from the parser,
    //! it is important that all nodes are allocated from a single, contiguous block of memory.
    //! Otherwise, cache misses when jumping between two (or more) disjoint blocks of memory can slow down parsing quite considerably.
    //! If required, you can tweak <code>RAPIDXML_STATIC_POOL_SIZE</code>, <code>RAPIDXML_DYNAMIC_POOL_SIZE</code> and <code>RAPIDXML_ALIGNMENT</code>
    //! to obtain best wasted memory to performance compromise.
    //! To do it, define their values before rapidxml.hpp file is included.
    //! \param Ch Character type of created nodes.
    template<class Ch = char>
    class memory_pool
    {

    public:

        //! \cond internal
        typedef void *(alloc_function)(std::size_t);       // Type of user-defined function used to allocate memory
        typedef void (free_func)(void *);              // Type of user-defined function used to free memory
        //! \endcond

        //! Constructs empty pool with default allocator functions.
        memory_pool()
            : m_alloc_function(0)
            , m_free_func(0)
        {
            init();
        }

        //! Destroys pool and frees all the memory.
        //! This causes memory occupied by nodes allocated by the pool to be freed.
        //! Nodes allocated from the pool are no longer valid.
        ~memory_pool()
        {
            clear();
        }

        //! Allocates a new node from the pool, and optionally assigns name and value to it.
        //! If the allocation request cannot be accomodated, this function will throw <code>std::bad_alloc</code>.
        //! If exceptions are disabled by defining RAPIDXML_NO_EXCEPTIONS, this function
        //! will call rapidxml::parse_error_handler() function.
        //! \param type Type of node to create.
        //! \param name Name to assign to the node, or 0 to assign no name.
        //! \param value Value to assign to the node, or 0 to assign no value.
        //! \param name_size Size of name to assign, or 0 to automatically calculate size from name string.
        //! \param value_size Size of value to assign, or 0 to automatically calculate size from value string.
        //! \return Pointer to allocated node. This pointer will never be NULL.
        xml_node<Ch> *allocate_node(node_type type,
                                    const Ch *name = 0, const Ch *value = 0,
                                    std::size_t name_size = 0, std::size_t value_size = 0)
        {
            void *memory = allocate_aligned(sizeof(xml_node<Ch>));
            xml_node<Ch> *node = new(memory) xml_node<Ch>(type);
            if (name)
            {
                if (name_size > 0)
                    node->name(name, name_size);
                else
                    node->name(name);
            }
            if (value)
            {
                if (value_size > 0)
                    node->value(value, value_size);
                else
                    node->value(value);
            }
            return node;
        }

        //! Allocates a new attribute from the pool, and optionally assigns name and value to it.
        //! If the allocation request cannot be accomodated, this function will throw <code>std::bad_alloc</code>.
        //! If exceptions are disabled by defining RAPIDXML_NO_EXCEPTIONS, this function
        //! will call rapidxml::parse_error_handler() function.
        //! \param name Name to assign to the attribute, or 0 to assign no name.
        //! \param value Value to assign to the attribute, or 0 to assign no value.
        //! \param name_size Size of name to assign, or 0 to automatically calculate size from name string.
        //! \param value_size Size of value to assign, or 0 to automatically calculate size from value string.
        //! \return Pointer to allocated attribute. This pointer will never be NULL.
        xml_attribute<Ch> *allocate_attribute(const Ch *name = 0, const Ch *value = 0,
                                              std::size_t name_size = 0, std::size_t value_size = 0)
        {
            void *memory = allocate_aligned(sizeof(xml_attribute<Ch>));
            xml_attribute<Ch> *attribute = new(memory) xml_attribute<Ch>;
            if (name)
            {
                if (name_size > 0)
                    attribute->name(name, name_size);
                else
                    attribute->name(name);
            }
            if (value)
            {
                if (value_size > 0)
                    attribute->value(value, value_size);
                else
                    attribute->value(value);
            }
            return attribute;
        }

        //! Allocates a char array of given size from the pool, and optionally copies a given string to it.
        //! If the allocation request cannot be accomodated, this function will throw <code>std::bad_alloc</code>.
        //! If exceptions are disabled by defining RAPIDXML_NO_EXCEPTIONS, this function
        //! will call rapidxml::parse_error_handler() function.
        //! \param source String to initialize the allocated memory with, or 0 to not initialize it.
        //! \param size Number of characters to allocate, or zero to calculate it automatically from source string length; if size is 0, source string must be specified and null terminated.
        //! \return Pointer to allocated char array. This pointer will never be NULL.
        Ch *allocate_string(const Ch *source = 0, std::size_t size = 0)
        {
            assert(source || size);     // Either source or size (or both) must be specified
            if (size == 0)
                size = internal::measure(source) + 1;
            Ch *result = static_cast<Ch *>(allocate_aligned(size * sizeof(Ch)));
            if (source)
                for (std::size_t i = 0; i < size; ++i)
                    result[i] = source[i];
            return result;
        }

        //! Clones an xml_node and its hierarchy of child nodes and attributes.
        //! Nodes and attributes are allocated from this memory pool.
        //! Names and values are not cloned, they are shared between the clone and the source.
        //! Result node can be optionally specified as a second parameter,
        //! in which case its contents will be replaced with cloned source node.
        //! This is useful when you want to clone entire document.
        //! \param source Node to clone.
        //! \param result Node to put results in, or 0 to automatically allocate result node
        //! \return Pointer to cloned node. This pointer will never be NULL.
        xml_node<Ch> *clone_node(const xml_node<Ch> *source, xml_node<Ch> *result = 0)
        {
            // Prepare result node
            if (result)
            {
                result->remove_all_attributes();
                result->remove_all_nodes();
                result->type(source->type());
            }
            else
                result = allocate_node(source->type());

            // Clone name and value
            result->name(source->name(), source->name_size());
            result->value(source->value(), source->value_size());

            // Clone child nodes and attributes
            for (xml_node<Ch> *child = source->first_node(); child; child = child->next_sibling())
                result->append_node(clone_node(child));
            for (xml_attribute<Ch> *attr = source->first_attribute(); attr; attr = attr->next_attribute())
                result->append_attribute(allocate_attribute(attr->name(), attr->value(), attr->name_size(), attr->value_size()));

            return result;
        }

        //! Clears the pool.
        //! This causes memory occupied by nodes allocated by the pool to be freed.
        //! Any nodes or strings allocated from the pool will no longer be valid.
        void clear()
        {
            while (m_begin != m_static_memory)
            {
                char *previous_begin = reinterpret_cast<header *>(align(m_begin))->previous_begin;
                if (m_free_func)
                    m_free_func(m_begin);
                else
                    delete[] m_begin;
                m_begin = previous_begin;
            }
            init();
        }

        //! Sets or resets the user-defined memory allocation functions for the pool.
        //! This can only be called when no memory is allocated from the pool yet, otherwise results are undefined.
        //! Allocation function must not return invalid pointer on failure. It should either throw,
        //! stop the program, or use <code>longjmp()</code> function to pass control to other place of program.
        //! If it returns invalid pointer, results are undefined.
        //! <br><br>
        //! User defined allocation functions must have the following forms:
        //! <br><code>
        //! <br>void *allocate(std::size_t size);
        //! <br>void free(void *pointer);
        //! </code><br>
        //! \param af Allocation function, or 0 to restore default function
        //! \param ff Free function, or 0 to restore default function
        void set_allocator(alloc_function *af, free_func *ff)
        {
            assert(m_begin == m_static_memory && m_ptr == align(m_begin));    // Verify that no memory is allocated yet
            m_alloc_function = af;
            m_free_func = ff;
        }

    private:

        struct header
        {
            char *previous_begin;
        };

        void init()
        {
            m_begin = m_static_memory;
            m_ptr = align(m_begin);
            m_end = m_static_memory + sizeof(m_static_memory);
        }

        char *align(char *ptr)
        {
            std::size_t alignment = ((RAPIDXML_ALIGNMENT - (std::size_t(ptr) & (RAPIDXML_ALIGNMENT - 1))) & (RAPIDXML_ALIGNMENT - 1));
            return ptr + alignment;
        }

        char *allocate_raw(std::size_t size)
        {
            // Allocate
            void *memory;
            if (m_alloc_function)   // Allocate memory using either user-specified allocation function or global operator new[]
            {
                memory = m_alloc_function(size);
                assert(memory); // Allocator is not allowed to return 0, on failure it must either throw, stop the program or use longjmp
            }
            else
            {
                memory = new char[size];
#ifdef RAPIDXML_NO_EXCEPTIONS
                if (!memory)            // If exceptions are disabled, verify memory allocation, because new will not be able to throw bad_alloc
                    RAPIDXML_PARSE_ERROR("out of memory", 0);
#endif
            }
            return static_cast<char *>(memory);
        }

        void *allocate_aligned(std::size_t size)
        {
            // Calculate aligned pointer
            char *result = align(m_ptr);

            // If not enough memory left in current pool, allocate a new pool
            if (result + size > m_end)
            {
                // Calculate required pool size (may be bigger than RAPIDXML_DYNAMIC_POOL_SIZE)
                std::size_t pool_size = RAPIDXML_DYNAMIC_POOL_SIZE;
                if (pool_size < size)
                    pool_size = size;

                // Allocate
                std::size_t alloc_size = sizeof(header) + (2 * RAPIDXML_ALIGNMENT - 2) + pool_size;     // 2 alignments required in worst case: one for header, one for actual allocation
                char *raw_memory = allocate_raw(alloc_size);

                // Setup new pool in allocated memory
                char *pool = align(raw_memory);
                header *new_header = reinterpret_cast<header *>(pool);
                new_header->previous_begin = m_begin;
                m_begin = raw_memory;
                m_ptr = pool + sizeof(header);
                m_end = raw_memory + alloc_size;

                // Calculate aligned pointer again using new pool
                result = align(m_ptr);
            }

            // Update pool and return aligned pointer
            m_ptr = result + size;
            return result;
        }

        char *m_begin;                                      // Start of raw memory making up current pool
        char *m_ptr;                                        // First free byte in current pool
        char *m_end;                                        // One past last available byte in current pool
        char m_static_memory[RAPIDXML_STATIC_POOL_SIZE];    // Static raw memory
        alloc_function *m_alloc_function;                           // Allocator function, or 0 if default is to be used
        free_func *m_free_func;                             // Free function, or 0 if default is to be used
    };

    ///////////////////////////////////////////////////////////////////////////
    // XML base

    //! Base class for xml_node and xml_attribute implementing common functions:
    //! name(), name_size(), value(), value_size() and parent().
    //! \param Ch Character type to use
    template<class Ch = char>
    class xml_base
    {

    public:

        ///////////////////////////////////////////////////////////////////////////
        // Construction & destruction

        // Construct a base with empty name, value and parent
        xml_base()
            : m_name(0)
            , m_value(0)
            , m_parent(0)
        {
        }

        ///////////////////////////////////////////////////////////////////////////
        // Node data access

        //! Gets name of the node.
        //! Interpretation of name depends on type of node.
        //! Note that name will not be zero-terminated if rapidxml::parse_no_string_terminators option was selected during parse.
        //! <br><br>
        //! Use name_size() function to determine length of the name.
        //! \return Name of node, or empty string if node has no name.
        Ch *name() const
        {
            return m_name ? m_name : nullstr();
        }

        //! Gets size of node name, not including terminator character.
        //! This function works correctly irrespective of whether name is or is not zero terminated.
        //! \return Size of node name, in characters.
        std::size_t name_size() const
        {
            return m_name ? m_name_size : 0;
        }

        //! Gets value of node.
        //! Interpretation of value depends on type of node.
        //! Note that value will not be zero-terminated if rapidxml::parse_no_string_terminators option was selected during parse.
        //! <br><br>
        //! Use value_size() function to determine length of the value.
        //! \return Value of node, or empty string if node has no value.
        Ch *value() const
        {
            return m_value ? m_value : nullstr();
        }

        //! Gets size of node value, not including terminator character.
        //! This function works correctly irrespective of whether value is or is not zero terminated.
        //! \return Size of node value, in characters.
        std::size_t value_size() const
        {
            return m_value ? m_value_size : 0;
        }

        ///////////////////////////////////////////////////////////////////////////
        // Node modification

        //! Sets name of node to a non zero-terminated string.
        //! See \ref ownership_of_strings.
        //! <br><br>
        //! Note that node does not own its name or value, it only stores a pointer to it.
        //! It will not delete or otherwise free the pointer on destruction.
        //! It is reponsibility of the user to properly manage lifetime of the string.
        //! The easiest way to achieve it is to use memory_pool of the document to allocate the string -
        //! on destruction of the document the string will be automatically freed.
        //! <br><br>
        //! Size of name must be specified separately, because name does not have to be zero terminated.
        //! Use name(const Ch *) function to have the length automatically calculated (string must be zero terminated).
        //! \param name Name of node to set. Does not have to be zero terminated.
        //! \param size Size of name, in characters. This does not include zero terminator, if one is present.
        void name(const Ch *name, std::size_t size)
        {
            m_name = const_cast<Ch *>(name);
            m_name_size = size;
        }

        //! Sets name of node to a zero-terminated string.
        //! See also \ref ownership_of_strings and xml_node::name(const Ch *, std::size_t).
        //! \param name Name of node to set. Must be zero terminated.
        void name(const Ch *name)
        {
            this->name(name, internal::measure(name));
        }

        //! Sets value of node to a non zero-terminated string.
        //! See \ref ownership_of_strings.
        //! <br><br>
        //! Note that node does not own its name or value, it only stores a pointer to it.
        //! It will not delete or otherwise free the pointer on destruction.
        //! It is reponsibility of the user to properly manage lifetime of the string.
        //! The easiest way to achieve it is to use memory_pool of the document to allocate the string -
        //! on destruction of the document the string will be automatically freed.
        //! <br><br>
        //! Size of value must be specified separately, because it does not have to be zero terminated.
        //! Use value(const Ch *) function to have the length automatically calculated (string must be zero terminated).
        //! <br><br>
        //! If an element has a child node of type node_data, it will take precedence over element value when printing.
        //! If you want to manipulate data of elements using values, use parser flag rapidxml::parse_no_data_nodes to prevent creation of data nodes by the parser.
        //! \param value value of node to set. Does not have to be zero terminated.
        //! \param size Size of value, in characters. This does not include zero terminator, if one is present.
        void value(const Ch *value, std::size_t size)
        {
            m_value = const_cast<Ch *>(value);
            m_value_size = size;
        }

        //! Sets value of node to a zero-terminated string.
        //! See also \ref ownership_of_strings and xml_node::value(const Ch *, std::size_t).
        //! \param value Vame of node to set. Must be zero terminated.
        void value(const Ch *value)
        {
            this->value(value, internal::measure(value));
        }

        ///////////////////////////////////////////////////////////////////////////
        // Related nodes access

        //! Gets node parent.
        //! \return Pointer to parent node, or 0 if there is no parent.
        xml_node<Ch> *parent() const
        {
            return m_parent;
        }

    protected:

        // Return empty string
        static Ch *nullstr()
        {
            static Ch zero = Ch('\0');
            return &zero;
        }

        Ch *m_name;                         // Name of node, or 0 if no name
        Ch *m_value;                        // Value of node, or 0 if no value
        std::size_t m_name_size;            // Length of node name, or undefined of no name
        std::size_t m_value_size;           // Length of node value, or undefined if no value
        xml_node<Ch> *m_parent;             // Pointer to parent node, or 0 if none

    };

    //! Class representing attribute node of XML document.
    //! Each attribute has name and value strings, which are available through name() and value() functions (inherited from xml_base).
    //! Note that after parse, both name and value of attribute will point to interior of source text used for parsing.
    //! Thus, this text must persist in memory for the lifetime of attribute.
    //! \param Ch Character type to use.
    template<class Ch = char>
    class xml_attribute: public xml_base<Ch>
    {

        friend class xml_node<Ch>;

    public:

        ///////////////////////////////////////////////////////////////////////////
        // Construction & destruction

        //! Constructs an empty attribute with the specified type.
        //! Consider using memory_pool of appropriate xml_document if allocating attributes manually.
        xml_attribute()
        {
        }

        ///////////////////////////////////////////////////////////////////////////
        // Related nodes access

        //! Gets document of which attribute is a child.
        //! \return Pointer to document that contains this attribute, or 0 if there is no parent document.
        xml_document<Ch> *document() const
        {
            if (xml_node<Ch> *node = this->parent())
            {
                while (node->parent())
                    node = node->parent();
                return node->type() == node_document ? static_cast<xml_document<Ch> *>(node) : 0;
            }
            else
                return 0;
        }

        //! Gets previous attribute, optionally matching attribute name.
        //! \param name Name of attribute to find, or 0 to return previous attribute regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found attribute, or 0 if not found.
        xml_attribute<Ch> *previous_attribute(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_attribute<Ch> *attribute = m_prev_attribute; attribute; attribute = attribute->m_prev_attribute)
                    if (internal::compare(attribute->name(), attribute->name_size(), name, name_size, case_sensitive))
                        return attribute;
                return 0;
            }
            else
                return this->m_parent ? m_prev_attribute : 0;
        }

        //! Gets next attribute, optionally matching attribute name.
        //! \param name Name of attribute to find, or 0 to return next attribute regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found attribute, or 0 if not found.
        xml_attribute<Ch> *next_attribute(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_attribute<Ch> *attribute = m_next_attribute; attribute; attribute = attribute->m_next_attribute)
                    if (internal::compare(attribute->name(), attribute->name_size(), name, name_size, case_sensitive))
                        return attribute;
                return 0;
            }
            else
                return this->m_parent ? m_next_attribute : 0;
        }

    private:

        xml_attribute<Ch> *m_prev_attribute;        // Pointer to previous sibling of attribute, or 0 if none; only valid if parent is non-zero
        xml_attribute<Ch> *m_next_attribute;        // Pointer to next sibling of attribute, or 0 if none; only valid if parent is non-zero

    };

    ///////////////////////////////////////////////////////////////////////////
    // XML node

    //! Class representing a node of XML document.
    //! Each node may have associated name and value strings, which are available through name() and value() functions.
    //! Interpretation of name and value depends on type of the node.
    //! Type of node can be determined by using type() function.
    //! <br><br>
    //! Note that after parse, both name and value of node, if any, will point interior of source text used for parsing.
    //! Thus, this text must persist in the memory for the lifetime of node.
    //! \param Ch Character type to use.
    template<class Ch = char>
    class xml_node: public xml_base<Ch>
    {

    public:

        ///////////////////////////////////////////////////////////////////////////
        // Construction & destruction

        //! Constructs an empty node with the specified type.
        //! Consider using memory_pool of appropriate document to allocate nodes manually.
        //! \param type Type of node to construct.
        xml_node(node_type type)
            : m_type(type)
            , m_first_node(0)
            , m_first_attribute(0)
        {
        }

        ///////////////////////////////////////////////////////////////////////////
        // Node data access

        //! Gets type of node.
        //! \return Type of node.
        node_type type() const
        {
            return m_type;
        }

        ///////////////////////////////////////////////////////////////////////////
        // Related nodes access

        //! Gets document of which node is a child.
        //! \return Pointer to document that contains this node, or 0 if there is no parent document.
        xml_document<Ch> *document() const
        {
            xml_node<Ch> *node = const_cast<xml_node<Ch> *>(this);
            while (node->parent())
                node = node->parent();
            return node->type() == node_document ? static_cast<xml_document<Ch> *>(node) : 0;
        }

        //! Gets first child node, optionally matching node name.
        //! \param name Name of child to find, or 0 to return first child regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found child, or 0 if not found.
        xml_node<Ch> *first_node(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_node<Ch> *child = m_first_node; child; child = child->next_sibling())
                    if (internal::compare(child->name(), child->name_size(), name, name_size, case_sensitive))
                        return child;
                return 0;
            }
            else
                return m_first_node;
        }

        //! Gets last child node, optionally matching node name.
        //! Behaviour is undefined if node has no children.
        //! Use first_node() to test if node has children.
        //! \param name Name of child to find, or 0 to return last child regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found child, or 0 if not found.
        xml_node<Ch> *last_node(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            assert(m_first_node);  // Cannot query for last child if node has no children
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_node<Ch> *child = m_last_node; child; child = child->previous_sibling())
                    if (internal::compare(child->name(), child->name_size(), name, name_size, case_sensitive))
                        return child;
                return 0;
            }
            else
                return m_last_node;
        }

        //! Gets previous sibling node, optionally matching node name.
        //! Behaviour is undefined if node has no parent.
        //! Use parent() to test if node has a parent.
        //! \param name Name of sibling to find, or 0 to return previous sibling regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found sibling, or 0 if not found.
        xml_node<Ch> *previous_sibling(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            assert(this->m_parent);     // Cannot query for siblings if node has no parent
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_node<Ch> *sibling = m_prev_sibling; sibling; sibling = sibling->m_prev_sibling)
                    if (internal::compare(sibling->name(), sibling->name_size(), name, name_size, case_sensitive))
                        return sibling;
                return 0;
            }
            else
                return m_prev_sibling;
        }

        //! Gets next sibling node, optionally matching node name.
        //! Behaviour is undefined if node has no parent.
        //! Use parent() to test if node has a parent.
        //! \param name Name of sibling to find, or 0 to return next sibling regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found sibling, or 0 if not found.
        xml_node<Ch> *next_sibling(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            assert(this->m_parent);     // Cannot query for siblings if node has no parent
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_node<Ch> *sibling = m_next_sibling; sibling; sibling = sibling->m_next_sibling)
                    if (internal::compare(sibling->name(), sibling->name_size(), name, name_size, case_sensitive))
                        return sibling;
                return 0;
            }
            else
                return m_next_sibling;
        }

        //! Gets first attribute of node, optionally matching attribute name.
        //! \param name Name of attribute to find, or 0 to return first attribute regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found attribute, or 0 if not found.
        xml_attribute<Ch> *first_attribute(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_attribute<Ch> *attribute = m_first_attribute; attribute; attribute = attribute->m_next_attribute)
                    if (internal::compare(attribute->name(), attribute->name_size(), name, name_size, case_sensitive))
                        return attribute;
                return 0;
            }
            else
                return m_first_attribute;
        }

        //! Gets last attribute of node, optionally matching attribute name.
        //! \param name Name of attribute to find, or 0 to return last attribute regardless of its name; this string doesn't have to be zero-terminated if name_size is non-zero
        //! \param name_size Size of name, in characters, or 0 to have size calculated automatically from string
        //! \param case_sensitive Should name comparison be case-sensitive; non case-sensitive comparison works properly only for ASCII characters
        //! \return Pointer to found attribute, or 0 if not found.
        xml_attribute<Ch> *last_attribute(const Ch *name = 0, std::size_t name_size = 0, bool case_sensitive = true) const
        {
            if (name)
            {
                if (name_size == 0)
                    name_size = internal::measure(name);
                for (xml_attribute<Ch> *attribute = m_last_attribute; attribute; attribute = attribute->m_prev_attribute)
                    if (internal::compare(attribute->name(), attribute->name_size(), name, name_size, case_sensitive))
                        return attribute;
                return 0;
            }
            else
                return m_first_attribute ? m_last_attribute : 0;
        }

        ///////////////////////////////////////////////////////////////////////////
        // Node modification

        //! Sets type of node.
        //! \param type Type of node to set.
        void type(node_type type)
        {
            m_type = type;
        }

        ///////////////////////////////////////////////////////////////////////////
        // Node manipulation

        //! Prepends a new child node.
        //! The prepended child becomes the first child, and all existing children are moved one position back.
        //! \param child Node to prepend.
        void prepend_node(xml_node<Ch> *child)
        {
            assert(child && !child->parent() && child->type() != node_document);
            if (first_node())
            {
                child->m_next_sibling = m_first_node;
                m_first_node->m_prev_sibling = child;
            }
            else
            {
                child->m_next_sibling = 0;
                m_last_node = child;
            }
            m_first_node = child;
            child->m_parent = this;
            child->m_prev_sibling = 0;
        }

        //! Appends a new child node.
        //! The appended child becomes the last child.
        //! \param child Node to append.
        void append_node(xml_node<Ch> *child)
        {
            assert(child && !child->parent() && child->type() != node_document);
            if (first_node())
            {
                child->m_prev_sibling = m_last_node;
                m_last_node->m_next_sibling = child;
            }
            else
            {
                child->m_prev_sibling = 0;
                m_first_node = child;
            }
            m_last_node = child;
            child->m_parent = this;
            child->m_next_sibling = 0;
        }

        //! Inserts a new child node at specified place inside the node.
        //! All children after and including the specified node are moved one position back.
        //! \param where Place where to insert the child, or 0 to insert at the back.
        //! \param child Node to insert.
        void insert_node(xml_node<Ch> *where, xml_node<Ch> *child)
        {
            assert(!where || where->parent() == this);
            assert(child && !child->parent() && child->type() != node_document);
            if (where == m_first_node)
                prepend_node(child);
            else if (where == 0)
                append_node(child);
            else
            {
                child->m_prev_sibling = where->m_prev_sibling;
                child->m_next_sibling = where;
                where->m_prev_sibling->m_next_sibling = child;
                where->m_prev_sibling = child;
                child->m_parent = this;
            }
        }

        //! Removes first child node.
        //! If node has no children, behaviour is undefined.
        //! Use first_node() to test if node has children.
        void remove_first_node()
        {
            assert(first_node());
            xml_node<Ch> *child = m_first_node;
            m_first_node = child->m_next_sibling;
            if (child->m_next_sibling)
                child->m_next_sibling->m_prev_sibling = 0;
            else
                m_last_node = 0;
            child->m_parent = 0;
        }

        //! Removes last child of the node.
        //! If node has no children, behaviour is undefined.
        //! Use first_node() to test if node has children.
        void remove_last_node()
        {
            assert(first_node());
            xml_node<Ch> *child = m_last_node;
            if (child->m_prev_sibling)
            {
                m_last_node = child->m_prev_sibling;
                child->m_prev_sibling->m_next_sibling = 0;
            }
            else
                m_first_node = 0;
            child->m_parent = 0;
        }

        //! Removes specified child from the node
        // \param where Pointer to child to be removed.
        void remove_node(xml_node<Ch> *where)
        {
            assert(where && where->parent() == this);
            assert(first_node());
            if (where == m_first_node)
                remove_first_node();
            else if (where == m_last_node)
                remove_last_node();
            else
            {
                where->m_prev_sibling->m_next_sibling = where->m_next_sibling;
                where->m_next_sibling->m_prev_sibling = where->m_prev_sibling;
                where->m_parent = 0;
            }
        }

        //! Removes all child nodes (but not attributes).
        void remove_all_nodes()
        {
            for (xml_node<Ch> *node = first_node(); node; node = node->m_next_sibling)
                node->m_parent = 0;
            m_first_node = 0;
        }

        //! Prepends a new attribute to the node.
        //! \param attribute Attribute to prepend.
        void prepend_attribute(xml_attribute<Ch> *attribute)
        {
            assert(attribute && !attribute->parent());
            if (first_attribute())
            {
                attribute->m_next_attribute = m_first_attribute;
                m_first_attribute->m_prev_attribute = attribute;
            }
            else
            {
                attribute->m_next_attribute = 0;
                m_last_attribute = attribute;
            }
            m_first_attribute = attribute;
            attribute->m_parent = this;
            attribute->m_prev_attribute = 0;
        }

        //! Appends a new attribute to the node.
        //! \param attribute Attribute to append.
        void append_attribute(xml_attribute<Ch> *attribute)
        {
            assert(attribute && !attribute->parent());
            if (first_attribute())
            {
                attribute->m_prev_attribute = m_last_attribute;
                m_last_attribute->m_next_attribute = attribute;
            }
            else
            {
                attribute->m_prev_attribute = 0;
                m_first_attribute = attribute;
            }
            m_last_attribute = attribute;
            attribute->m_parent = this;
            attribute->m_next_attribute = 0;
        }

        //! Inserts a new attribute at specified place inside the node.
        //! All attributes after and including the specified attribute are moved one position back.
        //! \param where Place where to insert the attribute, or 0 to insert at the back.
        //! \param attribute Attribute to insert.
        void insert_attribute(xml_attribute<Ch> *where, xml_attribute<Ch> *attribute)
        {
            assert(!where || where->parent() == this);
            assert(attribute && !attribute->parent());
            if (where == m_first_attribute)
                prepend_attribute(attribute);
            else if (where == 0)
                append_attribute(attribute);
            else
            {
                attribute->m_prev_attribute = where->m_prev_attribute;
                attribute->m_next_attribute = where;
                where->m_prev_attribute->m_next_attribute = attribute;
                where->m_prev_attribute = attribute;
                attribute->m_parent = this;
            }
        }

        //! Removes first attribute of the node.
        //! If node has no attributes, behaviour is undefined.
        //! Use first_attribute() to test if node has attributes.
        void remove_first_attribute()
        {
            assert(first_attribute());
            xml_attribute<Ch> *attribute = m_first_attribute;
            if (attribute->m_next_attribute)
            {
                attribute->m_next_attribute->m_prev_attribute = 0;
            }
            else
                m_last_attribute = 0;
            attribute->m_parent = 0;
            m_first_attribute = attribute->m_next_attribute;
        }

        //! Removes last attribute of the node.
        //! If node has no attributes, behaviour is undefined.
        //! Use first_attribute() to test if node has attributes.
        void remove_last_attribute()
        {
            assert(first_attribute());
            xml_attribute<Ch> *attribute = m_last_attribute;
            if (attribute->m_prev_attribute)
            {
                attribute->m_prev_attribute->m_next_attribute = 0;
                m_last_attribute = attribute->m_prev_attribute;
            }
            else
                m_first_attribute = 0;
            attribute->m_parent = 0;
        }

        //! Removes specified attribute from node.
        //! \param where Pointer to attribute to be removed.
        void remove_attribute(xml_attribute<Ch> *where)
        {
            assert(first_attribute() && where->parent() == this);
            if (where == m_first_attribute)
                remove_first_attribute();
            else if (where == m_last_attribute)
                remove_last_attribute();
            else
            {
                where->m_prev_attribute->m_next_attribute = where->m_next_attribute;
                where->m_next_attribute->m_prev_attribute = where->m_prev_attribute;
                where->m_parent = 0;
            }
        }

        //! Removes all attributes of node.
        void remove_all_attributes()
        {
            for (xml_attribute<Ch> *attribute = first_attribute(); attribute; attribute = attribute->m_next_attribute)
                attribute->m_parent = 0;
            m_first_attribute = 0;
        }

    private:

        ///////////////////////////////////////////////////////////////////////////
        // Restrictions

        // No copying
        xml_node(const xml_node &);
        void operator =(const xml_node &);

        ///////////////////////////////////////////////////////////////////////////
        // Data members

        // Note that some of the pointers below have UNDEFINED values if certain other pointers are 0.
        // This is required for maximum performance, as it allows the parser to omit initialization of
        // unneded/redundant values.
        //
        // The rules are as follows:
        // 1. first_node and first_attribute contain valid pointers, or 0 if node has no children/attributes respectively
        // 2. last_node and last_attribute are valid only if node has at least one child/attribute respectively, otherwise they contain garbage
        // 3. prev_sibling and next_sibling are valid only if node has a parent, otherwise they contain garbage

        node_type m_type;                       // Type of node; always valid
        xml_node<Ch> *m_first_node;             // Pointer to first child node, or 0 if none; always valid
        xml_node<Ch> *m_last_node;              // Pointer to last child node, or 0 if none; this value is only valid if m_first_node is non-zero
        xml_attribute<Ch> *m_first_attribute;   // Pointer to first attribute of node, or 0 if none; always valid
        xml_attribute<Ch> *m_last_attribute;    // Pointer to last attribute of node, or 0 if none; this value is only valid if m_first_attribute is non-zero
        xml_node<Ch> *m_prev_sibling;           // Pointer to previous sibling of node, or 0 if none; this value is only valid if m_parent is non-zero
        xml_node<Ch> *m_next_sibling;           // Pointer to next sibling of node, or 0 if none; this value is only valid if m_parent is non-zero

    };

    ///////////////////////////////////////////////////////////////////////////
    // XML document

    //! This class represents root of the DOM hierarchy.
    //! It is also an xml_node and a memory_pool through public inheritance.
    //! Use parse() function to build a DOM tree from a zero-terminated XML text string.
    //! parse() function allocates memory for nodes and attributes by using functions of xml_document,
    //! which are inherited from memory_pool.
    //! To access root node of the document, use the document itself, as if it was an xml_node.
    //! \param Ch Character type to use.
    template<class Ch = char>
    class xml_document: public xml_node<Ch>, public memory_pool<Ch>
    {

    public:

        //! Constructs empty XML document
        xml_document()
            : xml_node<Ch>(node_document)
        {
        }

        //! Parses zero-terminated XML string according to given flags.
        //! Passed string will be modified by the parser, unless rapidxml::parse_non_destructive flag is used.
        //! The string must persist for the lifetime of the document.
        //! In case of error, rapidxml::parse_error exception will be thrown.
        //! <br><br>
        //! If you want to parse contents of a file, you must first load the file into the memory, and pass pointer to its beginning.
        //! Make sure that data is zero-terminated.
        //! <br><br>
        //! Document can be parsed into multiple times.
        //! Each new call to parse removes previous nodes and attributes (if any), but does not clear memory pool.
        //! \param text XML data to parse; pointer is non-const to denote fact that this data may be modified by the parser.
        template<int Flags>
        void parse(Ch *text)
        {
            assert(text);

            // Remove current contents
            this->remove_all_nodes();
            this->remove_all_attributes();

            // Parse BOM, if any
            parse_bom<Flags>(text);

            // Parse children
            while (1)
            {
                // Skip whitespace before node
                skip<whitespace_pred, Flags>(text);
                if (*text == 0)
                    break;

                // Parse and append new child
                if (*text == Ch('<'))
                {
                    ++text;     // Skip '<'
                    if (xml_node<Ch> *node = parse_node<Flags>(text))
                        this->append_node(node);
                }
                else
                    RAPIDXML_PARSE_ERROR("expected <", text);
            }

        }

        //! Clears the document by deleting all nodes and clearing the memory pool.
        //! All nodes owned by document pool are destroyed.
        void clear()
        {
            this->remove_all_nodes();
            this->remove_all_attributes();
            memory_pool<Ch>::clear();
        }

    private:

        ///////////////////////////////////////////////////////////////////////
        // Internal character utility functions

        // Detect whitespace character
        struct whitespace_pred
        {
            static unsigned char test(Ch ch)
            {
                return internal::lookup_tables<0>::lookup_whitespace[static_cast<unsigned char>(ch)];
            }
        };

        // Detect node name character
        struct node_name_pred
        {
            static unsigned char test(Ch ch)
            {
                return internal::lookup_tables<0>::lookup_node_name[static_cast<unsigned char>(ch)];
            }
        };

        // Detect attribute name character
        struct attribute_name_pred
        {
            static unsigned char test(Ch ch)
            {
                return internal::lookup_tables<0>::lookup_attribute_name[static_cast<unsigned char>(ch)];
            }
        };

        // Detect text character (PCDATA)
        struct text_pred
        {
            static unsigned char test(Ch ch)
            {
                return internal::lookup_tables<0>::lookup_text[static_cast<unsigned char>(ch)];
            }
        };

        // Detect text character (PCDATA) that does not require processing
        struct text_pure_no_ws_pred
        {
            static unsigned char test(Ch ch)
            {
                return internal::lookup_tables<0>::lookup_text_pure_no_ws[static_cast<unsigned char>(ch)];
            }
        };

        // Detect text character (PCDATA) that does not require processing
        struct text_pure_with_ws_pred
        {
            static unsigned char test(Ch ch)
            {
                return internal::lookup_tables<0>::lookup_text_pure_with_ws[static_cast<unsigned char>(ch)];
            }
        };

        // Detect attribute value character
        template<Ch Quote>
        struct attribute_value_pred
        {
            static unsigned char test(Ch ch)
            {
                if (Quote == Ch('\''))
                    return internal::lookup_tables<0>::lookup_attribute_data_1[static_cast<unsigned char>(ch)];
                if (Quote == Ch('\"'))
                    return internal::lookup_tables<0>::lookup_attribute_data_2[static_cast<unsigned char>(ch)];
                return 0;       // Should never be executed, to avoid warnings on Comeau
            }
        };

        // Detect attribute value character
        template<Ch Quote>
        struct attribute_value_pure_pred
        {
            static unsigned char test(Ch ch)
            {
                if (Quote == Ch('\''))
                    return internal::lookup_tables<0>::lookup_attribute_data_1_pure[static_cast<unsigned char>(ch)];
                if (Quote == Ch('\"'))
                    return internal::lookup_tables<0>::lookup_attribute_data_2_pure[static_cast<unsigned char>(ch)];
                return 0;       // Should never be executed, to avoid warnings on Comeau
            }
        };

        // Insert coded character, using UTF8 or 8-bit ASCII
        template<int Flags>
        static void insert_coded_character(Ch *&text, unsigned long code)
        {
            if (Flags & parse_no_utf8)
            {
                // Insert 8-bit ASCII character
                // Todo: possibly verify that code is less than 256 and use replacement char otherwise?
                text[0] = static_cast<unsigned char>(code);
                text += 1;
            }
            else
            {
                // Insert UTF8 sequence
                if (code < 0x80)    // 1 byte sequence
                {
                    text[0] = static_cast<unsigned char>(code);
                    text += 1;
                }
                else if (code < 0x800)  // 2 byte sequence
                {
                    text[1] = static_cast<unsigned char>((code | 0x80) & 0xBF); code >>= 6;
                    text[0] = static_cast<unsigned char>(code | 0xC0);
                    text += 2;
                }
                else if (code < 0x10000)    // 3 byte sequence
                {
                    text[2] = static_cast<unsigned char>((code | 0x80) & 0xBF); code >>= 6;
                    text[1] = static_cast<unsigned char>((code | 0x80) & 0xBF); code >>= 6;
                    text[0] = static_cast<unsigned char>(code | 0xE0);
                    text += 3;
                }
                else if (code < 0x110000)   // 4 byte sequence
                {
                    text[3] = static_cast<unsigned char>((code | 0x80) & 0xBF); code >>= 6;
                    text[2] = static_cast<unsigned char>((code | 0x80) & 0xBF); code >>= 6;
                    text[1] = static_cast<unsigned char>((code | 0x80) & 0xBF); code >>= 6;
                    text[0] = static_cast<unsigned char>(code | 0xF0);
                    text += 4;
                }
                else    // Invalid, only codes up to 0x10FFFF are allowed in Unicode
                {
                    RAPIDXML_PARSE_ERROR("invalid numeric character entity", text);
                }
            }
        }

        // Skip characters until predicate evaluates to true
        template<class StopPred, int Flags>
        static void skip(Ch *&text)
        {
            Ch *tmp = text;
            while (StopPred::test(*tmp))
                ++tmp;
            text = tmp;
        }

        // Skip characters until predicate evaluates to true while doing the following:
        // - replacing XML character entity references with proper characters (&apos; &amp; &quot; &lt; &gt; &#...;)
        // - condensing whitespace sequences to single space character
        template<class StopPred, class StopPredPure, int Flags>
        static Ch *skip_and_expand_character_refs(Ch *&text)
        {
            // If entity translation, whitespace condense and whitespace trimming is disabled, use plain skip
            if (Flags & parse_no_entity_translation &&
                !(Flags & parse_normalize_whitespace) &&
                !(Flags & parse_trim_whitespace))
            {
                skip<StopPred, Flags>(text);
                return text;
            }

            // Use simple skip until first modification is detected
            skip<StopPredPure, Flags>(text);

            // Use translation skip
            Ch *src = text;
            Ch *dest = src;
            while (StopPred::test(*src))
            {
                // If entity translation is enabled
                if (!(Flags & parse_no_entity_translation))
                {
                    // Test if replacement is needed
                    if (src[0] == Ch('&'))
                    {
                        switch (src[1])
                        {

                        // &amp; &apos;
                        case Ch('a'):
                            if (src[2] == Ch('m') && src[3] == Ch('p') && src[4] == Ch(';'))
                            {
                                *dest = Ch('&');
                                ++dest;
                                src += 5;
                                continue;
                            }
                            if (src[2] == Ch('p') && src[3] == Ch('o') && src[4] == Ch('s') && src[5] == Ch(';'))
                            {
                                *dest = Ch('\'');
                                ++dest;
                                src += 6;
                                continue;
                            }
                            break;

                        // &quot;
                        case Ch('q'):
                            if (src[2] == Ch('u') && src[3] == Ch('o') && src[4] == Ch('t') && src[5] == Ch(';'))
                            {
                                *dest = Ch('"');
                                ++dest;
                                src += 6;
                                continue;
                            }
                            break;

                        // &gt;
                        case Ch('g'):
                            if (src[2] == Ch('t') && src[3] == Ch(';'))
                            {
                                *dest = Ch('>');
                                ++dest;
                                src += 4;
                                continue;
                            }
                            break;

                        // &lt;
                        case Ch('l'):
                            if (src[2] == Ch('t') && src[3] == Ch(';'))
                            {
                                *dest = Ch('<');
                                ++dest;
                                src += 4;
                                continue;
                            }
                            break;

                        // &#...; - assumes ASCII
                        case Ch('#'):
                            if (src[2] == Ch('x'))
                            {
                                unsigned long code = 0;
                                src += 3;   // Skip &#x
                                while (1)
                                {
                                    unsigned char digit = internal::lookup_tables<0>::lookup_digits[static_cast<unsigned char>(*src)];
                                    if (digit == 0xFF)
                                        break;
                                    code = code * 16 + digit;
                                    ++src;
                                }
                                insert_coded_character<Flags>(dest, code);    // Put character in output
                            }
                            else
                            {
                                unsigned long code = 0;
                                src += 2;   // Skip &#
                                while (1)
                                {
                                    unsigned char digit = internal::lookup_tables<0>::lookup_digits[static_cast<unsigned char>(*src)];
                                    if (digit == 0xFF)
                                        break;
                                    code = code * 10 + digit;
                                    ++src;
                                }
                                insert_coded_character<Flags>(dest, code);    // Put character in output
                            }
                            if (*src == Ch(';'))
                                ++src;
                            else
                                RAPIDXML_PARSE_ERROR("expected ;", src);
                            continue;

                        // Something else
                        default:
                            // Ignore, just copy '&' verbatim
                            break;

                        }
                    }
                }

                // If whitespace condensing is enabled
                if (Flags & parse_normalize_whitespace)
                {
                    // Test if condensing is needed
                    if (whitespace_pred::test(*src))
                    {
                        *dest = Ch(' '); ++dest;    // Put single space in dest
                        ++src;                      // Skip first whitespace char
                        // Skip remaining whitespace chars
                        while (whitespace_pred::test(*src))
                            ++src;
                        continue;
                    }
                }

                // No replacement, only copy character
                *dest++ = *src++;

            }

            // Return new end
            text = src;
            return dest;

        }

        ///////////////////////////////////////////////////////////////////////
        // Internal parsing functions

        // Parse BOM, if any
        template<int Flags>
        void parse_bom(Ch *&text)
        {
            // UTF-8?
            if (static_cast<unsigned char>(text[0]) == 0xEF &&
                static_cast<unsigned char>(text[1]) == 0xBB &&
                static_cast<unsigned char>(text[2]) == 0xBF)
            {
                text += 3;      // Skup utf-8 bom
            }
        }

        // Parse XML declaration (<?xml...)
        template<int Flags>
        xml_node<Ch> *parse_xml_declaration(Ch *&text)
        {
            // If parsing of declaration is disabled
            if (!(Flags & parse_declaration_node))
            {
                // Skip until end of declaration
                while (text[0] != Ch('?') || text[1] != Ch('>'))
                {
                    if (!text[0])
                        RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                    ++text;
                }
                text += 2;    // Skip '?>'
                return 0;
            }

            // Create declaration
            xml_node<Ch> *declaration = this->allocate_node(node_declaration);

            // Skip whitespace before attributes or ?>
            skip<whitespace_pred, Flags>(text);

            // Parse declaration attributes
            parse_node_attributes<Flags>(text, declaration);

            // Skip ?>
            if (text[0] != Ch('?') || text[1] != Ch('>'))
                RAPIDXML_PARSE_ERROR("expected ?>", text);
            text += 2;

            return declaration;
        }

        // Parse XML comment (<!--...)
        template<int Flags>
        xml_node<Ch> *parse_comment(Ch *&text)
        {
            // If parsing of comments is disabled
            if (!(Flags & parse_comment_nodes))
            {
                // Skip until end of comment
                while (text[0] != Ch('-') || text[1] != Ch('-') || text[2] != Ch('>'))
                {
                    if (!text[0])
                        RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                    ++text;
                }
                text += 3;     // Skip '-->'
                return 0;      // Do not produce comment node
            }

            // Remember value start
            Ch *value = text;

            // Skip until end of comment
            while (text[0] != Ch('-') || text[1] != Ch('-') || text[2] != Ch('>'))
            {
                if (!text[0])
                    RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                ++text;
            }

            // Create comment node
            xml_node<Ch> *comment = this->allocate_node(node_comment);
            comment->value(value, text - value);

            // Place zero terminator after comment value
            if (!(Flags & parse_no_string_terminators))
                *text = Ch('\0');

            text += 3;     // Skip '-->'
            return comment;
        }

        // Parse DOCTYPE
        template<int Flags>
        xml_node<Ch> *parse_doctype(Ch *&text)
        {
            // Remember value start
            Ch *value = text;

            // Skip to >
            while (*text != Ch('>'))
            {
                // Determine character type
                switch (*text)
                {

                // If '[' encountered, scan for matching ending ']' using naive algorithm with depth
                // This works for all W3C test files except for 2 most wicked
                case Ch('['):
                {
                    ++text;     // Skip '['
                    int depth = 1;
                    while (depth > 0)
                    {
                        switch (*text)
                        {
                            case Ch('['): ++depth; break;
                            case Ch(']'): --depth; break;
                            case 0: RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                        }
                        ++text;
                    }
                    break;
                }

                // Error on end of text
                case Ch('\0'):
                    RAPIDXML_PARSE_ERROR("unexpected end of data", text);

                // Other character, skip it
                default:
                    ++text;

                }
            }

            // If DOCTYPE nodes enabled
            if (Flags & parse_doctype_node)
            {
                // Create a new doctype node
                xml_node<Ch> *doctype = this->allocate_node(node_doctype);
                doctype->value(value, text - value);

                // Place zero terminator after value
                if (!(Flags & parse_no_string_terminators))
                    *text = Ch('\0');

                text += 1;      // skip '>'
                return doctype;
            }
            else
            {
                text += 1;      // skip '>'
                return 0;
            }

        }

        // Parse PI
        template<int Flags>
        xml_node<Ch> *parse_pi(Ch *&text)
        {
            // If creation of PI nodes is enabled
            if (Flags & parse_pi_nodes)
            {
                // Create pi node
                xml_node<Ch> *pi = this->allocate_node(node_pi);

                // Extract PI target name
                Ch *name = text;
                skip<node_name_pred, Flags>(text);
                if (text == name)
                    RAPIDXML_PARSE_ERROR("expected PI target", text);
                pi->name(name, text - name);

                // Skip whitespace between pi target and pi
                skip<whitespace_pred, Flags>(text);

                // Remember start of pi
                Ch *value = text;

                // Skip to '?>'
                while (text[0] != Ch('?') || text[1] != Ch('>'))
                {
                    if (*text == Ch('\0'))
                        RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                    ++text;
                }

                // Set pi value (verbatim, no entity expansion or whitespace normalization)
                pi->value(value, text - value);

                // Place zero terminator after name and value
                if (!(Flags & parse_no_string_terminators))
                {
                    pi->name()[pi->name_size()] = Ch('\0');
                    pi->value()[pi->value_size()] = Ch('\0');
                }

                text += 2;                          // Skip '?>'
                return pi;
            }
            else
            {
                // Skip to '?>'
                while (text[0] != Ch('?') || text[1] != Ch('>'))
                {
                    if (*text == Ch('\0'))
                        RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                    ++text;
                }
                text += 2;    // Skip '?>'
                return 0;
            }
        }

        // Parse and append data
        // Return character that ends data.
        // This is necessary because this character might have been overwritten by a terminating 0
        template<int Flags>
        Ch parse_and_append_data(xml_node<Ch> *node, Ch *&text, Ch *contents_start)
        {
            // Backup to contents start if whitespace trimming is disabled
            if (!(Flags & parse_trim_whitespace))
                text = contents_start;

            // Skip until end of data
            Ch *value = text, *end;
            if (Flags & parse_normalize_whitespace)
                end = skip_and_expand_character_refs<text_pred, text_pure_with_ws_pred, Flags>(text);
            else
                end = skip_and_expand_character_refs<text_pred, text_pure_no_ws_pred, Flags>(text);

            // Trim trailing whitespace if flag is set; leading was already trimmed by whitespace skip after >
            if (Flags & parse_trim_whitespace)
            {
                if (Flags & parse_normalize_whitespace)
                {
                    // Whitespace is already condensed to single space characters by skipping function, so just trim 1 char off the end
                    if (*(end - 1) == Ch(' '))
                        --end;
                }
                else
                {
                    // Backup until non-whitespace character is found
                    while (whitespace_pred::test(*(end - 1)))
                        --end;
                }
            }

            // If characters are still left between end and value (this test is only necessary if normalization is enabled)
            // Create new data node
            if (!(Flags & parse_no_data_nodes))
            {
                xml_node<Ch> *data = this->allocate_node(node_data);
                data->value(value, end - value);
                node->append_node(data);
            }

            // Add data to parent node if no data exists yet
            if (!(Flags & parse_no_element_values))
                if (*node->value() == Ch('\0'))
                    node->value(value, end - value);

            // Place zero terminator after value
            if (!(Flags & parse_no_string_terminators))
            {
                Ch ch = *text;
                *end = Ch('\0');
                return ch;      // Return character that ends data; this is required because zero terminator overwritten it
            }

            // Return character that ends data
            return *text;
        }

        // Parse CDATA
        template<int Flags>
        xml_node<Ch> *parse_cdata(Ch *&text)
        {
            // If CDATA is disabled
            if (Flags & parse_no_data_nodes)
            {
                // Skip until end of cdata
                while (text[0] != Ch(']') || text[1] != Ch(']') || text[2] != Ch('>'))
                {
                    if (!text[0])
                        RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                    ++text;
                }
                text += 3;      // Skip ]]>
                return 0;       // Do not produce CDATA node
            }

            // Skip until end of cdata
            Ch *value = text;
            while (text[0] != Ch(']') || text[1] != Ch(']') || text[2] != Ch('>'))
            {
                if (!text[0])
                    RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                ++text;
            }

            // Create new cdata node
            xml_node<Ch> *cdata = this->allocate_node(node_cdata);
            cdata->value(value, text - value);

            // Place zero terminator after value
            if (!(Flags & parse_no_string_terminators))
                *text = Ch('\0');

            text += 3;      // Skip ]]>
            return cdata;
        }

        // Parse element node
        template<int Flags>
        xml_node<Ch> *parse_element(Ch *&text)
        {
            // Create element node
            xml_node<Ch> *element = this->allocate_node(node_element);

            // Extract element name
            Ch *name = text;
            skip<node_name_pred, Flags>(text);
            if (text == name)
                RAPIDXML_PARSE_ERROR("expected element name", text);
            element->name(name, text - name);

            // Skip whitespace between element name and attributes or >
            skip<whitespace_pred, Flags>(text);

            // Parse attributes, if any
            parse_node_attributes<Flags>(text, element);

            // Determine ending type
            if (*text == Ch('>'))
            {
                ++text;
                parse_node_contents<Flags>(text, element);
            }
            else if (*text == Ch('/'))
            {
                ++text;
                if (*text != Ch('>'))
                    RAPIDXML_PARSE_ERROR("expected >", text);
                ++text;
            }
            else
                RAPIDXML_PARSE_ERROR("expected >", text);

            // Place zero terminator after name
            if (!(Flags & parse_no_string_terminators))
                element->name()[element->name_size()] = Ch('\0');

            // Return parsed element
            return element;
        }

        // Determine node type, and parse it
        template<int Flags>
        xml_node<Ch> *parse_node(Ch *&text)
        {
            // Parse proper node type
            switch (text[0])
            {

            // <...
            default:
                // Parse and append element node
                return parse_element<Flags>(text);

            // <?...
            case Ch('?'):
                ++text;     // Skip ?
                if ((text[0] == Ch('x') || text[0] == Ch('X')) &&
                    (text[1] == Ch('m') || text[1] == Ch('M')) &&
                    (text[2] == Ch('l') || text[2] == Ch('L')) &&
                    whitespace_pred::test(text[3]))
                {
                    // '<?xml ' - xml declaration
                    text += 4;      // Skip 'xml '
                    return parse_xml_declaration<Flags>(text);
                }
                else
                {
                    // Parse PI
                    return parse_pi<Flags>(text);
                }

            // <!...
            case Ch('!'):

                // Parse proper subset of <! node
                switch (text[1])
                {

                // <!-
                case Ch('-'):
                    if (text[2] == Ch('-'))
                    {
                        // '<!--' - xml comment
                        text += 3;     // Skip '!--'
                        return parse_comment<Flags>(text);
                    }
                    break;

                // <![
                case Ch('['):
                    if (text[2] == Ch('C') && text[3] == Ch('D') && text[4] == Ch('A') &&
                        text[5] == Ch('T') && text[6] == Ch('A') && text[7] == Ch('['))
                    {
                        // '<![CDATA[' - cdata
                        text += 8;     // Skip '![CDATA['
                        return parse_cdata<Flags>(text);
                    }
                    break;

                // <!D
                case Ch('D'):
                    if (text[2] == Ch('O') && text[3] == Ch('C') && text[4] == Ch('T') &&
                        text[5] == Ch('Y') && text[6] == Ch('P') && text[7] == Ch('E') &&
                        whitespace_pred::test(text[8]))
                    {
                        // '<!DOCTYPE ' - doctype
                        text += 9;      // skip '!DOCTYPE '
                        return parse_doctype<Flags>(text);
                    }

                }   // switch

                // Attempt to skip other, unrecognized node types starting with <!
                ++text;     // Skip !
                while (*text != Ch('>'))
                {
                    if (*text == 0)
                        RAPIDXML_PARSE_ERROR("unexpected end of data", text);
                    ++text;
                }
                ++text;     // Skip '>'
                return 0;   // No node recognized

            }
        }

        // Parse contents of the node - children, data etc.
        template<int Flags>
        void parse_node_contents(Ch *&text, xml_node<Ch> *node)
        {
            // For all children and text
            while (1)
            {
                // Skip whitespace between > and node contents
                Ch *contents_start = text;      // Store start of node contents before whitespace is skipped
                skip<whitespace_pred, Flags>(text);
                Ch next_char = *text;

            // After data nodes, instead of continuing the loop, control jumps here.
            // This is because zero termination inside parse_and_append_data() function
            // would wreak havoc with the above code.
            // Also, skipping whitespace after data nodes is unnecessary.
            after_data_node:

                // Determine what comes next: node closing, child node, data node, or 0?
                switch (next_char)
                {

                // Node closing or child node
                case Ch('<'):
                    if (text[1] == Ch('/'))
                    {
                        // Node closing
                        text += 2;      // Skip '</'
                        if (Flags & parse_validate_closing_tags)
                        {
                            // Skip and validate closing tag name
                            Ch *closing_name = text;
                            skip<node_name_pred, Flags>(text);
                            if (!internal::compare(node->name(), node->name_size(), closing_name, text - closing_name, true))
                                RAPIDXML_PARSE_ERROR("invalid closing tag name", text);
                        }
                        else
                        {
                            // No validation, just skip name
                            skip<node_name_pred, Flags>(text);
                        }
                        // Skip remaining whitespace after node name
                        skip<whitespace_pred, Flags>(text);
                        if (*text != Ch('>'))
                            RAPIDXML_PARSE_ERROR("expected >", text);
                        ++text;     // Skip '>'
                        return;     // Node closed, finished parsing contents
                    }
                    else
                    {
                        // Child node
                        ++text;     // Skip '<'
                        if (xml_node<Ch> *child = parse_node<Flags>(text))
                            node->append_node(child);
                    }
                    break;

                // End of data - error
                case Ch('\0'):
                    RAPIDXML_PARSE_ERROR("unexpected end of data", text);

                // Data node
                default:
                    next_char = parse_and_append_data<Flags>(node, text, contents_start);
                    goto after_data_node;   // Bypass regular processing after data nodes

                }
            }
        }

        // Parse XML attributes of the node
        template<int Flags>
        void parse_node_attributes(Ch *&text, xml_node<Ch> *node)
        {
            // For all attributes
            while (attribute_name_pred::test(*text))
            {
                // Extract attribute name
                Ch *name = text;
                ++text;     // Skip first character of attribute name
                skip<attribute_name_pred, Flags>(text);
                if (text == name)
                    RAPIDXML_PARSE_ERROR("expected attribute name", name);

                // Create new attribute
                xml_attribute<Ch> *attribute = this->allocate_attribute();
                attribute->name(name, text - name);
                node->append_attribute(attribute);

                // Skip whitespace after attribute name
                skip<whitespace_pred, Flags>(text);

                // Skip =
                if (*text != Ch('='))
                    RAPIDXML_PARSE_ERROR("expected =", text);
                ++text;

                // Add terminating zero after name
                if (!(Flags & parse_no_string_terminators))
                    attribute->name()[attribute->name_size()] = 0;

                // Skip whitespace after =
                skip<whitespace_pred, Flags>(text);

                // Skip quote and remember if it was ' or "
                Ch quote = *text;
                if (quote != Ch('\'') && quote != Ch('"'))
                    RAPIDXML_PARSE_ERROR("expected ' or \"", text);
                ++text;

                // Extract attribute value and expand char refs in it
                Ch *value = text, *end;
                const int AttFlags = Flags & ~parse_normalize_whitespace;   // No whitespace normalization in attributes
                if (quote == Ch('\''))
                    end = skip_and_expand_character_refs<attribute_value_pred<Ch('\'')>, attribute_value_pure_pred<Ch('\'')>, AttFlags>(text);
                else
                    end = skip_and_expand_character_refs<attribute_value_pred<Ch('"')>, attribute_value_pure_pred<Ch('"')>, AttFlags>(text);

                // Set attribute value
                attribute->value(value, end - value);

                // Make sure that end quote is present
                if (*text != quote)
                    RAPIDXML_PARSE_ERROR("expected ' or \"", text);
                ++text;     // Skip quote

                // Add terminating zero after value
                if (!(Flags & parse_no_string_terminators))
                    attribute->value()[attribute->value_size()] = 0;

                // Skip whitespace after attribute value
                skip<whitespace_pred, Flags>(text);
            }
        }

    };

    //! \cond internal
    namespace internal
    {

        // Whitespace (space \n \r \t)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_whitespace[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  0,  0,  1,  0,  0,  // 0
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 1
             1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 2
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 3
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 4
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 5
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 6
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 7
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 8
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // 9
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // A
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // B
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // C
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // D
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  // E
             0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0   // F
        };

        // Node name (anything but space \n \r \t / > ? \0)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_node_name[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  1,  1,  0,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Text (i.e. PCDATA) (anything but < \0)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_text[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Text (i.e. PCDATA) that does not require processing when ws normalization is disabled
        // (anything but < \0 &)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_text_pure_no_ws[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Text (i.e. PCDATA) that does not require processing when ws normalizationis is enabled
        // (anything but < \0 & space \n \r \t)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_text_pure_with_ws[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  1,  1,  0,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             0,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Attribute name (anything but space \n \r \t / < > = ? ! \0)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_attribute_name[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  1,  1,  0,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  0,  0,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Attribute data with single quote (anything but ' \0)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_attribute_data_1[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Attribute data with single quote that does not require processing (anything but ' \0 &)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_attribute_data_1_pure[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             1,  1,  1,  1,  1,  1,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Attribute data with double quote (anything but " \0)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_attribute_data_2[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Attribute data with double quote that does not require processing (anything but " \0 &)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_attribute_data_2_pure[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
             0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 0
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 1
             1,  1,  0,  1,  1,  1,  0,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 2
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 3
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 4
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 5
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 6
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 7
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 8
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // 9
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // A
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // B
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // C
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // D
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  // E
             1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1,  1   // F
        };

        // Digits (dec and hex, 255 denotes end of numeric character reference)
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_digits[256] =
        {
          // 0   1   2   3   4   5   6   7   8   9   A   B   C   D   E   F
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 0
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 1
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 2
             0,  1,  2,  3,  4,  5,  6,  7,  8,  9,255,255,255,255,255,255,  // 3
           255, 10, 11, 12, 13, 14, 15,255,255,255,255,255,255,255,255,255,  // 4
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 5
           255, 10, 11, 12, 13, 14, 15,255,255,255,255,255,255,255,255,255,  // 6
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 7
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 8
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // 9
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // A
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // B
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // C
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // D
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,  // E
           255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255   // F
        };

        // Upper case conversion
        template<int Dummy>
        const unsigned char lookup_tables<Dummy>::lookup_upcase[256] =
        {
          // 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  A   B   C   D   E   F
           0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,   // 0
           16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,   // 1
           32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,   // 2
           48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63,   // 3
           64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,   // 4
           80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,   // 5
           96, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,   // 6
           80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 123,124,125,126,127,  // 7
           128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,  // 8
           144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159,  // 9
           160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175,  // A
           176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191,  // B
           192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207,  // C
           208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223,  // D
           224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239,  // E
           240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255   // F
        };
    }
    //! \endcond

}

// Undefine internal macros
#undef RAPIDXML_PARSE_ERROR

// On MSVC, restore warnings state
#ifdef _MSC_VER
    #pragma warning(pop)
#endif

#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/third/xml/rapidxml.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/third/xml/xml.hpp
// //////////////////////////////////////////////////////////////////////

#ifndef ARBITER_IS_AMALGAMATION

#ifndef ARBITER_EXTERNAL_XML
#include <arbiter/third/xml/rapidxml.hpp>
#endif

#endif

#ifdef ARBITER_EXTERNAL_XML
#include <rapidxml.hpp>
#endif

namespace Xml = rapidxml;


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/third/xml/xml.hpp
// //////////////////////////////////////////////////////////////////////

#include <nlohmann/json.hpp>


// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/exports.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#if defined(_WIN32) || defined(WIN32) || defined(_MSC_VER)
#define ARBITER_WINDOWS
#endif

#ifndef ARBITER_DLL
#if defined(ARBITER_WINDOWS)
#if defined(ARBITER_DLL_EXPORT)
#   define ARBITER_DLL   __declspec(dllexport)
#elif defined(PDAL_DLL_IMPORT)
#   define ARBITER_DLL   __declspec(dllimport)
#else
#   define ARBITER_DLL
#endif
#else
#  if defined(USE_GCC_VISIBILITY_FLAG)
#    define ARBITER_DLL     __attribute__ ((visibility("default")))
#  else
#    define ARBITER_DLL
#  endif
#endif
#endif

#ifdef _WIN32
#pragma warning(disable:4251)// [templated class] needs to have dll-interface...
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/exports.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/types.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

using StringMap = std::map<std::string, std::string>;

/** @brief Exception class for all internally thrown runtime errors. */
class ArbiterError : public std::runtime_error
{
public:
    ArbiterError(std::string msg) : std::runtime_error(msg) { }
};

namespace http
{

/** HTTP header fields. */
using Headers = StringMap;

/** HTTP query parameters. */
using Query = StringMap;

/** @cond arbiter_internal */

class Response
{
public:
    Response(int code = 0)
        : m_code(code)
        , m_data()
    { }

    Response(int code, std::vector<char> data)
        : m_code(code)
        , m_data(data)
    { }

    Response(
            int code,
            const std::vector<char>& data,
            const Headers& headers)
        : m_code(code)
        , m_data(data)
        , m_headers(headers)
    { }

    ~Response() { }

    bool ok() const             { return m_code / 100 == 2; }
    bool clientError() const    { return m_code / 100 == 4; }
    bool serverError() const    { return m_code / 100 == 5; }
    int code() const            { return m_code; }

    std::vector<char> data() const { return m_data; }
    const Headers& headers() const { return m_headers; }
    std::string str() const
    {
        return std::string(data().data(), data().size());
    }

private:
    int m_code;
    std::vector<char> m_data;
    Headers m_headers;
};

/** @endcond */

} // namespace http
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/types.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/json.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/third/json/json.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

using json = nlohmann::json;

// Work around:
// https://github.com/nlohmann/json/issues/709
// https://github.com/google/googletest/pull/1186
inline void PrintTo(const json& j, std::ostream* os) { *os << j.dump(); }

// Merge B into A, without overwriting any keys from A.
inline json merge(const json& a, const json& b)
{
    json out(a);
    if (out.is_null()) out = json::object();

    if (!b.is_null())
    {
        if (b.is_object())
        {
            for (const auto& p : b.items())
            {
                // If A doesn't have this key, then set it to B's value.
                // If A has the key but it's an object, then recursively
                // merge.
                // Otherwise A already has a value here that we won't
                // overwrite.
                const std::string& key(p.key());
                const json& val(p.value());

                if (!out.count(key)) out[key] = val;
                else if (out[key].is_object()) merge(out[key], val);
            }
        }
        else
        {
            out = b;
        }
    }

    return out;
}

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/json.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/curl.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#include <arbiter/util/types.hpp>
#endif

#ifdef ARBITER_CURL
#include <curl/curl.h>
#else
typedef void CURL;
#endif

struct curl_slist;

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace http
{

/** @cond arbiter_internal */

class Pool;

class ARBITER_DLL Curl
{
    friend class Pool;

    static constexpr std::size_t defaultHttpTimeout = 5;

public:
    ~Curl();

    http::Response get(
            std::string path,
            Headers headers,
            Query query,
            std::size_t reserve);

    http::Response head(std::string path, Headers headers, Query query);

    http::Response put(
            std::string path,
            const std::vector<char>& data,
            Headers headers,
            Query query);

    http::Response post(
            std::string path,
            const std::vector<char>& data,
            Headers headers,
            Query query);

private:
    Curl(std::string j);

    void init(std::string path, const Headers& headers, const Query& query);

    // Returns HTTP status code.
    int perform();

    Curl(const Curl&);
    Curl& operator=(const Curl&);

    CURL* m_curl = nullptr;
    curl_slist* m_headers = nullptr;

    bool m_verbose = false;
    long m_timeout = defaultHttpTimeout;
    bool m_followRedirect = true;
    bool m_verifyPeer = true;
    std::unique_ptr<std::string> m_caPath;
    std::unique_ptr<std::string> m_caInfo;
    std::unique_ptr<std::string> m_proxy;

    std::vector<char> m_data;
};

/** @endcond */

} // namespace http
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/curl.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/http.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/curl.hpp>
#include <arbiter/util/exports.hpp>
#include <arbiter/util/types.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace http
{

/** Perform URI percent-encoding, without encoding characters included in
 * @p exclusions.
 */
ARBITER_DLL std::string sanitize(std::string path, std::string exclusions = "/");

/** Build a query string from key-value pairs.  If @p query is empty, the
 * result is an empty string.  Otherwise, the result will start with the
 * '?' character.
 */
ARBITER_DLL std::string buildQueryString(const http::Query& query);

/** @cond arbiter_internal */

class ARBITER_DLL Pool;

class ARBITER_DLL Resource
{
public:
    Resource(Pool& pool, Curl& curl, std::size_t id, std::size_t retry);
    ~Resource();

    http::Response get(
            std::string path,
            Headers headers = Headers(),
            Query query = Query(),
            std::size_t reserve = 0);

    http::Response head(
            std::string path,
            Headers headers = Headers(),
            Query query = Query());

    http::Response put(
            std::string path,
            const std::vector<char>& data,
            Headers headers = Headers(),
            Query query = Query());

    http::Response post(
            std::string path,
            const std::vector<char>& data,
            Headers headers = Headers(),
            Query query = Query());

private:
    Pool& m_pool;
    Curl& m_curl;
    std::size_t m_id;
    std::size_t m_retry;

    http::Response exec(std::function<http::Response()> f);
};

class ARBITER_DLL Pool
{
    // Only HttpResource may release.
    friend class Resource;

public:
    Pool() : Pool(4, 4, "") { }
    Pool(std::size_t concurrent, std::size_t retry, std::string j);
    ~Pool();

    Resource acquire();

private:
    void release(std::size_t id);

    std::vector<std::unique_ptr<Curl>> m_curls;
    std::vector<std::size_t> m_available;
    std::size_t m_retry;

    std::mutex m_mutex;
    std::condition_variable m_cv;
};

/** @endcond */

} // namespace http
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/http.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/ini.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <map>
#include <string>
#include <vector>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace ini
{

using Section = std::string;
using Key = std::string;
using Val = std::string;
using Contents = std::map<Section, std::map<Key, Val>>;

Contents parse(const std::string& s);

} // namespace ini

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/ini.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/time.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <ctime>
#include <string>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif



#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

class ARBITER_DLL Time
{
public:
    static const std::string iso8601;
    static const std::string rfc822;
    static const std::string iso8601NoSeparators;
    static const std::string dateNoSeparators;

    Time();
    Time(const std::string& s, const std::string& format = "%Y-%m-%dT%H:%M:%SZ");

    std::string str(const std::string& format = "%Y-%m-%dT%H:%M:%SZ") const;

    // Return value is in seconds.
    int64_t operator-(const Time& other) const;
    int64_t asUnix() const;

private:
    std::time_t m_time;
};

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/time.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/macros.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#define ROTLEFT(a,b) (((a) << (b)) | ((a) >> (32-(b))))
#define ROTRIGHT(a,b) (((a) >> (b)) | ((a) << (32-(b))))

// SHA256.
#define CH(x,y,z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTRIGHT(x,2) ^ ROTRIGHT(x,13) ^ ROTRIGHT(x,22))
#define EP1(x) (ROTRIGHT(x,6) ^ ROTRIGHT(x,11) ^ ROTRIGHT(x,25))
#define SIG0(x) (ROTRIGHT(x,7) ^ ROTRIGHT(x,18) ^ ((x) >> 3))
#define SIG1(x) (ROTRIGHT(x,17) ^ ROTRIGHT(x,19) ^ ((x) >> 10))

// MD5.
#define F(x,y,z) ((x & y) | (~x & z))
#define G(x,y,z) ((x & z) | (y & ~z))
#define H(x,y,z) (x ^ y ^ z)
#define I(x,y,z) (y ^ (x | ~z))

#define FF(a,b,c,d,m,s,t) { a += F(b,c,d) + m + t; \
                            a = b + ROTLEFT(a,s); }
#define GG(a,b,c,d,m,s,t) { a += G(b,c,d) + m + t; \
                            a = b + ROTLEFT(a,s); }
#define HH(a,b,c,d,m,s,t) { a += H(b,c,d) + m + t; \
                            a = b + ROTLEFT(a,s); }
#define II(a,b,c,d,m,s,t) { a += I(b,c,d) + m + t; \
                            a = b + ROTLEFT(a,s); }


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/macros.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/md5.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif

// MD5 implementation adapted from:
//      https://github.com/B-Con/crypto-algorithms

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace crypto
{

ARBITER_DLL std::string md5(const std::string& data);

} // namespace crypto
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/md5.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/sha256.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif


// SHA256 implementation adapted from:
//      https://github.com/B-Con/crypto-algorithms
// HMAC:
//      https://en.wikipedia.org/wiki/Hash-based_message_authentication_code

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace crypto
{

ARBITER_DLL std::vector<char> sha256(const std::vector<char>& data);
ARBITER_DLL std::string sha256(const std::string& data);

ARBITER_DLL std::string hmacSha256(
        const std::string& key,
        const std::string& data);

} // namespace crypto
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/sha256.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/transforms.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif


#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace crypto
{

ARBITER_DLL std::string encodeBase64(
        const std::vector<char>& data,
        bool pad = true);
ARBITER_DLL std::string encodeBase64(const std::string& data, bool pad = true);

ARBITER_DLL std::string decodeBase64(const std::string& data);

ARBITER_DLL std::string encodeAsHex(const std::vector<char>& data);
ARBITER_DLL std::string encodeAsHex(const std::string& data);

} // namespace crypto
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/transforms.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/util/util.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <algorithm>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

/** General utilities. */

/** Returns @p path, less any trailing glob indicators (one or two
 * asterisks) as well as any possible trailing slash.
 */
ARBITER_DLL std::string stripPostfixing(std::string path);

/** Returns the portion of @p fullPath following the last instance of the
 * character `/`, if any instances exist aside from possibly the delimiter
 * `://`.  If there are no other instances of `/`, then @p fullPath itself
 * will be returned.
 *
 * If @p fullPath ends with a trailing `/` or a glob indication (i.e. is a
 * directory), these trailing characters will be stripped prior to the
 * logic above, thus the innermost directory in the full path will be
 * returned.
 */
ARBITER_DLL std::string getBasename(std::string fullPath);

/** Returns everything besides the basename, as determined by `getBasename`.
 * For file paths, this corresponds to the directory path above the file.
 * For directory paths, this corresponds to all directories above the
 * innermost directory.
 */
ARBITER_DLL std::string getDirname(std::string fullPath);

/** @cond arbiter_internal */
ARBITER_DLL inline bool isSlash(char c) { return c == '/' || c == '\\'; }

/** Returns true if the last character is an asterisk. */
ARBITER_DLL inline bool isGlob(std::string path)
{
    return path.size() && path.back() == '*';
}

/** Returns true if the last character is a slash or an asterisk. */
inline bool isDirectory(std::string path)
{
    return (path.size() && isSlash(path.back())) || isGlob(path);
}

inline std::string joinImpl(bool first = false) { return std::string(); }

template <typename ...Paths>
inline std::string joinImpl(
        bool first,
        std::string current,
        Paths&&... paths)
{
    const bool currentIsDir(current.size() && isSlash(current.back()));
    std::string next(joinImpl(false, std::forward<Paths>(paths)...));

    // Strip slashes from the front of our remainder.
    while (next.size() && isSlash(next.front())) next = next.substr(1);

    if (first)
    {
        // If this is the first component, strip a single trailing slash if
        // one exists - but do not strip a double trailing slash since we
        // want to retain Windows paths like "C://".
        if (
                current.size() > 1 &&
                isSlash(current.back()) &&
                !isSlash(current.at(current.size() - 2)))
        {
            current.pop_back();
        }
    }
    else
    {
        while (current.size() && isSlash(current.back()))
        {
            current.pop_back();
        }
        if (current.empty()) return next;
    }

    std::string sep;

    if (next.size() && (current.empty() || !isSlash(current.back())))
    {
        // We are going to join current with a populated subpath, so make
        // sure they are separated by a slash.
#ifdef ARBITER_WINDOWS
        sep = "\\";
#else
        sep = "/";
#endif
    }
    else if (next.empty() && currentIsDir)
    {
        // We are at the end of the chain, and the last component was a
        // directory.  Retain its trailing slash.
        if (current.size() && !isSlash(current.back()))
        {
#ifdef ARBITER_WINDOWS
            sep = "\\";
#else
            sep = "/";
#endif

        }
    }

    return current + sep + next;
}
/** @endcond */

/** @brief Join one or more path components "intelligently".
 *
 * The result is the concatenation of @p path and any members of @p paths
 * with exactly one slash preceding each non-empty portion of @p path or
 * @p paths.  Portions of @p paths will be stripped of leading slashes prior
 * to processing, so portions containing only slashes are considered empty.
 *
 * If @p path contains a single trailing slash preceded by a non-slash
 * character, then that slash will be stripped prior to processing.
 *
 * @code
 * join("")                                 // ""
 * join("/")                                // "/"
 * join("/var", "log", "arbiter.log")       // "/var/log/arbiter.log"
 * join("/var/", "log", "arbiter.log")      // "/var/log/arbiter.log"
 * join("", "var", "log", "arbiter.log")    // "/var/log/arbiter.log"
 * join("/", "/var", "log", "arbiter.log")  // "/var/log/arbiter.log"
 * join("", "/var", "log", "arbiter.log")   // "/var/log/arbiter.log"
 * join("~", "code", "", "test.cpp", "/")   // "~/code/test.cpp"
 * join("C:\\", "My Documents")             // "C:/My Documents"
 * join("s3://", "bucket", "object.txt")    // "s3://bucket/object.txt"
 * @endcode
 */
template <typename ...Paths>
inline std::string join(std::string path, Paths&&... paths)
{
    return joinImpl(true, path, std::forward<Paths>(paths)...);
}

/** @brief Extract an environment variable, if it exists, independent of
 * platform.
 */
ARBITER_DLL std::unique_ptr<std::string> env(const std::string& var);

/** @brief Split a string on a token. */
ARBITER_DLL std::vector<std::string> split(
        const std::string& s,
        char delimiter = '\n');

/** @brief Remove whitespace. */
ARBITER_DLL std::string stripWhitespace(const std::string& s);

namespace internal
{

template<typename T, typename... Args>
std::unique_ptr<T> makeUnique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template<typename T>
std::unique_ptr<T> clone(const T& t)
{
    return makeUnique<T>(t);
}

template<typename T>
std::unique_ptr<T> maybeClone(const T* t)
{
    if (t) return makeUnique<T>(*t);
    else return std::unique_ptr<T>();
}

} // namespace internal

ARBITER_DLL uint64_t randomNumber();

/** If no delimiter of "://" is found, returns "file".  Otherwise, returns
 * the substring prior to but not including this delimiter.
 */
ARBITER_DLL std::string getProtocol(std::string path);

/** Strip the type and delimiter `://`, if they exist. */
ARBITER_DLL std::string stripProtocol(std::string path);

/** Get the characters following the final instance of '.', or an empty
 * string if there are no '.' characters. */
ARBITER_DLL std::string getExtension(std::string path);

/** Strip the characters following (and including) the final instance of
 * '.' if one exists, otherwise return the full path. */
ARBITER_DLL std::string stripExtension(std::string path);

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/util/util.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/driver.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

class HttpPool;

/** @brief Base class for interacting with a storage type.
 *
 * A Driver handles reading, writing, and possibly globbing from a storage
 * source.  It is intended to be overriden by specialized subclasses for each
 * supported storage mechanism.
 *
 * Derived classes must override Driver::type,
 * Driver::put(std::string, const std::vector<char>&) const, and
 * Driver::get(std::string, std::vector<char>&) const,
 * Driver::getSize(std::string) const - and may optionally
 * override Driver::glob if possible.
 *
 * HTTP-derived classes should override the PUT and GET versions that accept
 * http::Headers and http::Query parameters instead.
 */
class ARBITER_DLL Driver
{
public:
    virtual ~Driver() { }

    /**
     * Returns a string identifying this driver type, which should be unique
     * among all other drivers.  Paths that begin with the substring
     * `<type>://` will be routed to this driver.  For example, `fs`, `s3`, or
     * `http`.
     *
     * @note Derived classes must override.
     */
    virtual std::string type() const = 0;

    /** Get string data. */
    std::string get(std::string path) const;

    /** Get string data, if available. */
    std::unique_ptr<std::string> tryGet(std::string path) const;

    /** Get binary data. */
    std::vector<char> getBinary(std::string path) const;

    /** Get binary data, if available. */
    std::unique_ptr<std::vector<char>> tryGetBinary(std::string path) const;

    /**
     * Write @p data to the given @p path.
     *
     * @note Derived classes must override.
     *
     * @param path Path with the type-specifying prefix information stripped.
     */
    virtual void put(std::string path, const std::vector<char>& data) const = 0;

    /** True for remote paths, otherwise false.  If `true`, a fs::LocalHandle
     * request will download and write this file to the local filesystem.
     */
    virtual bool isRemote() const { return true; }

    /** Get the file size in bytes, if available. */
    virtual std::unique_ptr<std::size_t> tryGetSize(std::string path) const = 0;

    /** Get the file size in bytes, or throw if it does not exist. */
    std::size_t getSize(std::string path) const;

    /** Write string data. */
    void put(std::string path, const std::string& data) const;

    /** Copy a file, where @p src and @p dst must both be of this driver
     * type.  Type-prefixes must be stripped from the input parameters.
     */
    virtual void copy(std::string src, std::string dst) const;

    /** @brief Resolve a possibly globbed path.
     *
     * See Arbiter::resolve for details.
     */
    std::vector<std::string> resolve(
            std::string path,
            bool verbose = false) const;

protected:
    /** @brief Resolve a wildcard path.
     *
     * This operation should return a non-recursive resolution of the files
     * matching the given wildcard @p path (no directories).  With the exception
     * of the filesystem Driver, results should be prefixed with
     * `type() + "://"`.
     *
     * @note The default behavior is to throw ArbiterError, so derived classes
     * may optionally override if they can perform this behavior.
     *
     * @param path A string ending with the character `*`, and with all
     * type-specifying prefix information like `http://` or `s3://` stripped.
     *
     * @param verbose If true, this function may print out minimal information
     * to indicate that progress is occurring.
     */
    virtual std::vector<std::string> glob(std::string path, bool verbose) const;

    /**
     * @param path Path with the type-specifying prefix information stripped.
     * @param[out] data Empty vector in which to write resulting data.
     */
    virtual bool get(std::string path, std::vector<char>& data) const = 0;
};

typedef std::map<std::string, std::unique_ptr<Driver>> DriverMap;

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/driver.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/fs.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/driver.hpp>
#endif

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/exports.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

class Arbiter;
class Endpoint;

/**
 * \addtogroup fs
 * @{
 */

/** Filesystem utilities. */

/** @brief Returns true if created, false if already existed. */
ARBITER_DLL bool mkdirp(std::string dir);

/** @brief Returns true if removed, otherwise false. */
ARBITER_DLL bool remove(std::string filename);

/** @brief Performs tilde expansion to a fully-qualified path, if possible.
 */
ARBITER_DLL std::string expandTilde(std::string path);

/** @brief Get temporary path from environment. */
ARBITER_DLL std::string getTempPath();

/** @brief Resolve a possible wildcard path. */
ARBITER_DLL std::vector<std::string> glob(std::string path);

/** @brief A scoped local filehandle for a possibly remote path.
 *
 * This is an RAII style pseudo-filehandle.  It manages the scope of a
 * local temporary version of a file, where that file may have been copied
 * from a remote storage location.
 *
 * See Arbiter::getLocalHandle for details about construction.
 */
class ARBITER_DLL LocalHandle
{
public:
    LocalHandle(std::string localPath, bool isRemote);

    /** @brief Deletes the local path if the data was copied from a remote
     * source.
     *
     * This is a no-op if the path was already local and not copied.
     */
    ~LocalHandle();

    /** @brief Get the path of the locally stored file.
     *
     * @return A local filesystem absolute path containing the data
     * requested in Arbiter::getLocalHandle.
     */
    std::string localPath() const { return m_localPath; }

    /** @brief Release the managed local path and return the path from
     * LocalHandle::localPath.
     *
     * After this call, destruction of the LocalHandle will not erase the
     * temporary file that may have been created.
     */
    std::string release()
    {
        m_erase = false;
        return localPath();
    }

private:
    const std::string m_localPath;
    bool m_erase;
};
/** @} */

namespace drivers
{

/** @brief Local filesystem driver. */
class ARBITER_DLL Fs : public Driver
{
public:
    Fs() { }

    using Driver::get;

    static std::unique_ptr<Fs> create();

    virtual std::string type() const override { return "file"; }

    virtual std::unique_ptr<std::size_t> tryGetSize(
            std::string path) const override;

    virtual void put(
            std::string path,
            const std::vector<char>& data) const override;

    virtual std::vector<std::string> glob(
            std::string path,
            bool verbose) const override;

    virtual bool isRemote() const override { return false; }

    virtual void copy(std::string src, std::string dst) const override;

protected:
    virtual bool get(std::string path, std::vector<char>& data) const override;
};

} // namespace drivers

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/fs.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/http.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include <memory>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/driver.hpp>
#include <arbiter/util/http.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{
namespace drivers
{

/** @brief HTTP driver.  Intended as both a standalone driver as well as a base
 * for derived drivers build atop HTTP.
 *
 * Derivers should overload the HTTP-specific put/get methods that accept
 * headers and query parameters rather than Driver::put and Driver::get.
 *
 * Internal methods for derivers are provided as protected methods.
 */
class ARBITER_DLL Http : public Driver
{
public:
    Http(http::Pool& pool);
    static std::unique_ptr<Http> create(http::Pool& pool);

    // Inherited from Driver.
    virtual std::string type() const override { return "http"; }

    /** By default, performs a HEAD request and returns the contents of the
     * Content-Length header.
     */
    virtual std::unique_ptr<std::size_t> tryGetSize(
            std::string path) const override;

    virtual void put(
            std::string path,
            const std::vector<char>& data) const final override
    {
        put(path, data, http::Headers(), http::Query());
    }

    /* HTTP-specific driver methods follow.  Since many drivers (S3, Dropbox,
     * etc.) are built atop HTTP, we'll provide HTTP-specific methods for
     * derived classes to use in addition to the generic PUT/GET combinations.
     *
     * Specifically, we'll add POST/HEAD calls, and allow headers and query
     * parameters to be passed as well.
     */

    /** Perform an HTTP GET request. */
    std::string get(
            std::string path,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    /** Perform an HTTP GET request. */
    std::unique_ptr<std::string> tryGet(
            std::string path,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    /* Perform an HTTP HEAD request. */
    std::size_t getSize(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /* Perform an HTTP HEAD request. */
    std::unique_ptr<std::size_t> tryGetSize(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Perform an HTTP GET request. */
    std::vector<char> getBinary(
            std::string path,
            http::Headers headers,
            http::Query query) const;

    /** Perform an HTTP GET request. */
    std::unique_ptr<std::vector<char>> tryGetBinary(
            std::string path,
            http::Headers headers,
            http::Query query) const;

    /** Perform an HTTP PUT request. */
    void put(
            std::string path,
            const std::string& data,
            http::Headers headers,
            http::Query query) const;

    /** HTTP-derived Drivers should override this version of PUT to allow for
     * custom headers and query parameters.
     */

    /** Perform an HTTP PUT request. */
    virtual void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query) const;

    void post(
            std::string path,
            const std::string& data,
            http::Headers headers,
            http::Query query) const;
    void post(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query) const;

    /* These operations are other HTTP-specific calls that derived drivers may
     * need for their underlying API use.
     */
    http::Response internalGet(
            std::string path,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query(),
            std::size_t reserve = 0) const;

    http::Response internalPut(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    http::Response internalHead(
            std::string path,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    http::Response internalPost(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

protected:
    /** HTTP-derived Drivers should override this version of GET to allow for
     * custom headers and query parameters.
     */
    virtual bool get(
            std::string path,
            std::vector<char>& data,
            http::Headers headers,
            http::Query query) const;

    http::Pool& m_pool;

private:
    virtual bool get(
            std::string path,
            std::vector<char>& data) const final override
    {
        return get(path, data, http::Headers(), http::Query());
    }

    std::string typedPath(const std::string& p) const;
};

/** @brief HTTPS driver.  Identical to the HTTP driver except for its type
 * string.
 */
class Https : public Http
{
public:
    Https(http::Pool& pool) : Http(pool) { }

    static std::unique_ptr<Https> create(http::Pool& pool)
    {
        return std::unique_ptr<Https>(new Https(pool));
    }

    virtual std::string type() const override { return "https"; }
};

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/http.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/s3.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/time.hpp>
#include <arbiter/util/util.hpp>
#include <arbiter/drivers/http.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace drivers
{

/** @brief Amazon %S3 driver. */
class S3 : public Http
{
    class Auth;
    class AuthFields;
    class Config;

public:
    S3(
            http::Pool& pool,
            std::string profile,
            std::unique_ptr<Auth> auth,
            std::unique_ptr<Config> config);

    /** Try to construct an S3 driver.  The configuration/credential discovery
     * follows, in order:
     *      - Environment settings.
     *      - Arbiter JSON configuration.
     *      - Well-known files or their environment overrides, like
     *          `~/.aws/credentials` or the file at AWS_CREDENTIAL_FILE.
     *      - EC2 instance profile.
     */
    static std::vector<std::unique_ptr<S3>> create(
            http::Pool& pool,
            std::string j);

    static std::unique_ptr<S3> createOne(http::Pool& pool, std::string j);

    // Overrides.
    virtual std::string type() const override;

    virtual std::unique_ptr<std::size_t> tryGetSize(
            std::string path) const override;

    /** Inherited from Drivers::Http. */
    virtual void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

    virtual void copy(std::string src, std::string dst) const override;

private:
    static std::string extractProfile(std::string j);

    /*
    static std::unique_ptr<Config> extractConfig(
            std::string j,
            std::string profile);

            */
    /** Inherited from Drivers::Http. */
    virtual bool get(
            std::string path,
            std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

    virtual std::vector<std::string> glob(
            std::string path,
            bool verbose) const override;

    class ApiV4;
    class Resource;

    std::string m_profile;
    std::unique_ptr<Auth> m_auth;
    std::unique_ptr<Config> m_config;
};

class S3::AuthFields
{
public:
    AuthFields(std::string access, std::string hidden, std::string token = "")
        : m_access(access), m_hidden(hidden), m_token(token)
    { }

    const std::string& access() const { return m_access; }
    const std::string& hidden() const { return m_hidden; }
    const std::string& token() const { return m_token; }

private:
    std::string m_access;
    std::string m_hidden;
    std::string m_token;
};

class S3::Auth
{
public:
    Auth(std::string access, std::string hidden, std::string token = "")
        : m_access(access)
        , m_hidden(hidden)
        , m_token(token)
    { }

    Auth(std::string iamRole)
        : m_role(internal::makeUnique<std::string>(iamRole))
    { }

    static std::unique_ptr<Auth> create(std::string j, std::string profile);

    AuthFields fields() const;

private:
    mutable std::string m_access;
    mutable std::string m_hidden;
    mutable std::string m_token;

    std::unique_ptr<std::string> m_role;
    mutable std::unique_ptr<Time> m_expiration;
    mutable std::mutex m_mutex;
};

class S3::Config
{
public:
    Config(std::string j, std::string profile);

    const std::string& region() const { return m_region; }
    const std::string& baseUrl() const { return m_baseUrl; }
    const http::Headers& baseHeaders() const { return m_baseHeaders; }
    bool precheck() const { return m_precheck; }

private:
    static std::string extractRegion(std::string j, std::string profile);
    static std::string extractBaseUrl(std::string j, std::string region);

    const std::string m_region;
    const std::string m_baseUrl;
    http::Headers m_baseHeaders;
    bool m_precheck;
};



class S3::Resource
{
public:
    Resource(std::string baseUrl, std::string fullPath);

    std::string url() const;
    std::string host() const;
    std::string baseUrl() const;
    std::string bucket() const;
    std::string object() const;

private:
    std::string m_baseUrl;
    std::string m_bucket;
    std::string m_object;
    bool m_virtualHosted;
};

class S3::ApiV4
{
public:
    ApiV4(
            std::string verb,
            const std::string& region,
            const Resource& resource,
            const S3::AuthFields authFields,
            const http::Query& query,
            const http::Headers& headers,
            const std::vector<char>& data);

    const http::Headers& headers() const { return m_headers; }
    const http::Query& query() const { return m_query; }

    const std::string& signedHeadersString() const
    {
        return m_signedHeadersString;
    }

private:
    std::string buildCanonicalRequest(
            std::string verb,
            const Resource& resource,
            const http::Query& query,
            const std::vector<char>& data) const;

    std::string buildStringToSign(
            const std::string& canonicalRequest) const;

    std::string calculateSignature(
            const std::string& stringToSign) const;

    std::string getAuthHeader(
            const std::string& signedHeadersString,
            const std::string& signature) const;

    const S3::AuthFields m_authFields;
    const std::string m_region;
    const Time m_time;

    http::Headers m_headers;
    http::Query m_query;
    std::string m_canonicalHeadersString;
    std::string m_signedHeadersString;
};

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/s3.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/az.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/util/time.hpp>
#include <arbiter/util/util.hpp>
#include <arbiter/drivers/http.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace drivers
{

/** @brief Microsoft %Azure blob driver. */
class AZ : public Http
{
    class AuthFields;
    class Config;

public:
    AZ(
            http::Pool& pool,
            std::string profile,
            std::unique_ptr<Config> config);

    /** Try to construct an Azure blob driver.  The configuration/credential discovery
     * follows, in order:
     *      - Environment settings.
     *      - Arbiter JSON configuration.
     */
    static std::vector<std::unique_ptr<AZ>> create(
            http::Pool& pool,
            std::string j);

    static std::unique_ptr<AZ> createOne(http::Pool& pool, std::string j);

    // Overrides.
    virtual std::string type() const override;

    virtual std::unique_ptr<std::size_t> tryGetSize(
            std::string path) const override;

    /** Inherited from Drivers::Http. */
    virtual void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

    virtual void copy(std::string src, std::string dst) const override;

private:
    static std::string extractProfile(std::string j);

    /*
    static std::unique_ptr<Config> extractConfig(
            std::string j,
            std::string profile);

            */
    /** Inherited from Drivers::Http. */
    virtual bool get(
            std::string path,
            std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

    virtual std::vector<std::string> glob(
            std::string path,
            bool verbose) const override;

    class ApiV1;
    class Resource;

    std::string m_profile;
    std::unique_ptr<Config> m_config;
};

class AZ::AuthFields
{
public:
    AuthFields(std::string account, std::string key="")
        : m_storageAccount(account), m_storageAccessKey(key)
    { }

    const std::string& account() const { return m_storageAccount; }
    const std::string& key() const { return m_storageAccessKey; }

private:
    std::string m_storageAccount;
    std::string m_storageAccessKey;
};

class AZ::Config
{

public:
    Config(std::string j, std::string profile);

    const std::string& service() const { return m_service; }
    const std::string& storageAccount() const { return m_storageAccount; }
    const std::string& endpoint() const { return m_endpoint; }
    const std::string& baseUrl() const { return m_baseUrl; }
    const http::Headers& baseHeaders() const { return m_baseHeaders; }
    bool precheck() const { return m_precheck; }

    AuthFields authFields() const { return AuthFields(m_storageAccount, m_storageAccessKey); }

private:
    static std::string extractService(std::string j, std::string profile);
    static std::string extractEndpoint(std::string j, std::string profile);
    static std::string extractBaseUrl(std::string j, std::string endpoint, std::string service, std::string account);
    static std::string extractStorageAccount(std::string j, std::string profile);
    static std::string extractStorageAccessKey(std::string j, std::string profile);

    const std::string m_service;
    const std::string m_storageAccount;
    const std::string m_storageAccessKey;
    const std::string m_endpoint;
    const std::string m_baseUrl;
    http::Headers m_baseHeaders;
    bool m_precheck;
};



class AZ::Resource
{
public:
    Resource(std::string baseUrl, std::string fullPath);

    std::string url() const;
    std::string host() const;
    std::string baseUrl() const;
    std::string bucket() const;
    std::string object() const;
    std::string blob() const;
    std::string storageAccount() const;

private:
    std::string m_baseUrl;
    std::string m_bucket;
    std::string m_object;
    std::string m_storageAccount;
};

class AZ::ApiV1
{
public:
    ApiV1(
            std::string verb,
            const Resource& resource,
            const AZ::AuthFields authFields,
            const http::Query& query,
            const http::Headers& headers,
            const std::vector<char>& data);

    const http::Headers& headers() const { return m_headers; }
    const http::Query& query() const { return m_query; }

private:
    std::string buildCanonicalResource(
            const Resource& resource,
            const http::Query& query) const;

    std::string buildCanonicalHeader(
            http::Headers & msHeaders,
            const http::Headers & existingHeaders) const;

    std::string buildStringToSign(
            const std::string& verb,
            const http::Headers& headers,
            const std::string& canonicalHeaders,
            const std::string& canonicalRequest) const;

    std::string calculateSignature(
            const std::string& stringToSign) const;

    std::string getAuthHeader(
            const std::string& signedHeadersString) const;

    const AZ::AuthFields m_authFields;
    const Time m_time;

    http::Headers m_headers;
    http::Query m_query;
};

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/az.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/google.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/drivers/http.hpp>
#endif

#include <mutex>

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace drivers
{

class Google : public Https
{
    class Auth;
public:
    Google(http::Pool& pool, std::unique_ptr<Auth> auth);

    static std::unique_ptr<Google> create(http::Pool& pool, std::string j);

    // Overrides.
    virtual std::string type() const override { return "gs"; }  // Match gsutil.

    virtual std::unique_ptr<std::size_t> tryGetSize(
            std::string path) const override;

    /** Inherited from Drivers::Http. */
    virtual void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

private:
    /** Inherited from Drivers::Http. */
    virtual bool get(
            std::string path,
            std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

    virtual std::vector<std::string> glob(
            std::string path,
            bool verbose) const override;

    std::unique_ptr<Auth> m_auth;
};

class Google::Auth
{
public:
    Auth(std::string s);
    static std::unique_ptr<Auth> create(std::string s);

    http::Headers headers() const;

private:
    void maybeRefresh() const;
    std::string sign(std::string data, std::string privateKey) const;

    const std::string m_clientEmail;
    const std::string m_privateKey;
    mutable int64_t m_expiration = 0;   // Unix time.
    mutable http::Headers m_headers;

    mutable std::mutex m_mutex;
};

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/google.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/dropbox.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <memory>
#include <string>
#include <vector>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/drivers/http.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace drivers
{

/** @brief %Dropbox driver. */
class Dropbox : public Http
{
public:
    class Auth;
    Dropbox(http::Pool& pool, const Auth& auth);

    /** Try to construct a %Dropbox Driver.  @p j may be stringified JSON, in
     * which the key `token` will be used to construct the Dropbox auth, or
     * may simply be the token string itself.
     */
    static std::unique_ptr<Dropbox> create(http::Pool& pool, std::string j);

    virtual std::string type() const override { return "dropbox"; }
    virtual void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query = http::Query()) const override;

    /** @brief %Dropbox authentication information. */
    class Auth
    {
    public:
        explicit Auth(std::string token) : m_token(token) { }
        const std::string& token() const { return m_token; }

    private:
        std::string m_token;
    };

private:
    virtual bool get(
            std::string path,
            std::vector<char>& data,
            http::Headers headers,
            http::Query query) const override;

    virtual std::unique_ptr<std::size_t> tryGetSize(
            std::string path) const override;

    virtual std::vector<std::string> glob(
            std::string path,
            bool verbose) const override;

    std::string continueFileInfo(std::string cursor) const;

    http::Headers httpGetHeaders() const;
    http::Headers httpPostHeaders() const;

    Auth m_auth;
};

} // namespace drivers
} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/dropbox.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/drivers/test.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <numeric>

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/drivers/fs.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

class Arbiter;

namespace drivers
{

/** @brief A filesystem driver that acts as if it were remote for testing
 * purposes.
 */
class Test : public Fs
{
public:
    static std::unique_ptr<Test> create()
    {
        return std::unique_ptr<Test>(new Test());
    }

    virtual std::string type() const override { return "test"; }
    virtual bool isRemote() const override { return true; }

private:
    virtual std::vector<std::string> glob(
            std::string path,
            bool verbose) const override
    {
        auto results(Fs::glob(path, verbose));
        for (auto& p : results) p = type() + "://" + p;
        return results;
    }
};

} // namespace drivers

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/drivers/test.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/endpoint.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <string>
#include <vector>
#include <memory>

#ifndef ARBITER_IS_AMALGAMATION

#include <arbiter/drivers/fs.hpp>
#include <arbiter/util/exports.hpp>
#include <arbiter/util/types.hpp>

#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace drivers { class Http; }
namespace http { class Pool; }

class Driver;

/** @brief A utility class to drive usage from a common root directory.
 *
 * Acts as a reusable Driver based on a single root directory.  The interface
 * is the same as a Driver, although @p path parameters represent subpaths
 * which will be appended to the value from Endpoint::root to form a full
 * path.
 *
 * An Endpoint may be created using Arbiter::getEndpoint.
 */
class ARBITER_DLL Endpoint
{
    // Only Arbiter may construct.
    friend class Arbiter;

public:
    Endpoint() : m_driver(nullptr)
    {}

    /** Returns root directory name without any type-prefixing, and will
     * always end with the character `/`.  For example `~/data/`, or
     * `my-bucket/nested-directory/`.
     */
    std::string root() const;

    /** Returns root directory name ending with the character `/`.  If
     * `isRemote` is `true`, then the path will be prefixed with
     * `type() + "://"`.
     */
    std::string prefixedRoot() const;

    // Driver passthroughs.

    /** Passthrough to Driver::type. */
    std::string type() const;

    /** Passthrough to Driver::isRemote. */
    bool isRemote() const;

    /** Negation of Endpoint::isRemote. */
    bool isLocal() const;

    /** See Arbiter::isHttpDerived. */
    bool isHttpDerived() const;

    /** See Arbiter::getLocalHandle. */
    LocalHandle getLocalHandle(
            std::string subpath,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    /** Passthrough to Driver::get. */
    std::string get(std::string subpath) const;

    /** Passthrough to Driver::tryGet. */
    std::unique_ptr<std::string> tryGet(std::string subpath) const;

    /** Passthrough to Driver::getBinary. */
    std::vector<char> getBinary(std::string subpath) const;

    /** Passthrough to Driver::tryGetBinary. */
    std::unique_ptr<std::vector<char>> tryGetBinary(std::string subpath) const;

    /** Passthrough to Driver::getSize. */
    std::size_t getSize(std::string subpath) const;

    /** Passthrough to Driver::tryGetSize. */
    std::unique_ptr<std::size_t> tryGetSize(std::string subpath) const;

    /** Passthrough to Driver::put(std::string, const std::string&) const. */
    void put(std::string subpath, const std::string& data) const;

    /** Passthrough to
     * Driver::put(std::string, const std::vector<char>&) const.
     */
    void put(std::string subpath, const std::vector<char>& data) const;

    // HTTP-specific passthroughs.

    /** Passthrough to
     * drivers::Http::get(std::string, http::Headers, http::Query) const. */
    std::string get(
            std::string subpath,
            http::Headers headers,
            http::Query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::tryGet(std::string, http::Headers, http::Query) const. */
    std::unique_ptr<std::string> tryGet(
            std::string subpath,
            http::Headers headers,
            http::Query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::getBinary(std::string, http::Headers, http::Query) const.
     */
    std::vector<char> getBinary(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::tryGetBinary(std::string, http::Headers, http::Query) const.
     */
    std::unique_ptr<std::vector<char>> tryGetBinary(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::getSize(std::string, http::Headers, http::Query) const.
     */
    std::size_t getSize(
            std::string subpath,
            http::Headers headers,
            http::Query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::tryGetSize(std::string, http::Headers, http::Query) const.
     */
    std::unique_ptr<std::size_t> tryGetSize(
            std::string subpath,
            http::Headers headers,
            http::Query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::put(std::string, const std::string&, http::Headers, http::Query) const.
     */
    void put(
            std::string path,
            const std::string& data,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Passthrough to
     * drivers::Http::put(std::string, const std::vector<char>&, http::Headers, http::Query) const.
     */
    void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query = http::Query()) const;

    // Endpoint specifics.

    /** Get the full path corresponding to this subpath.  The path will not
     * be prefixed with the driver type or the `://` delimiter.
     */
    std::string fullPath(const std::string& subpath) const;

    /** Get the full path corresponding to this subpath.  If `isRemote` is
     * `true`, then the path will be prefixed with `type() + "://"`.
     */
    std::string prefixedFullPath(const std::string& subpath) const;

    /** Get a further nested subpath relative to this Endpoint's root. */
    Endpoint getSubEndpoint(std::string subpath) const;

    http::Response httpGet(
            std::string path,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query(),
            std::size_t reserve = 0) const;

    http::Response httpPut(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    http::Response httpHead(
            std::string path,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

    http::Response httpPost(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers = http::Headers(),
            http::Query query = http::Query()) const;

private:
    Endpoint(const Driver& driver, std::string root);

    // If `isRemote()`, returns the type and delimiter, otherwise returns an
    // empty string.
    std::string softPrefix() const;

    const drivers::Http* tryGetHttpDriver() const;
    const drivers::Http& getHttpDriver() const;

    const Driver* m_driver;
    std::string m_root;
};

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif


// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/endpoint.hpp
// //////////////////////////////////////////////////////////////////////






// //////////////////////////////////////////////////////////////////////
// Beginning of content of file: arbiter/arbiter.hpp
// //////////////////////////////////////////////////////////////////////

#pragma once

#include <vector>
#include <string>

#if defined(_WIN32) || defined(WIN32) || defined(_MSC_VER)
#define ARBITER_WINDOWS
#endif

#ifndef ARBITER_IS_AMALGAMATION
#include <arbiter/endpoint.hpp>
#include <arbiter/driver.hpp>
#include <arbiter/drivers/dropbox.hpp>
#include <arbiter/drivers/fs.hpp>
#include <arbiter/drivers/google.hpp>
#include <arbiter/drivers/http.hpp>
#include <arbiter/drivers/s3.hpp>
#include <arbiter/drivers/az.hpp>
#include <arbiter/drivers/test.hpp>
#include <arbiter/util/exports.hpp>
#include <arbiter/util/types.hpp>
#include <arbiter/util/util.hpp>
#endif

#ifdef ARBITER_CUSTOM_NAMESPACE
namespace ARBITER_CUSTOM_NAMESPACE
{
#endif

namespace arbiter
{

namespace http { class Pool; }

/** @brief The primary interface for storage abstraction.
 *
 * The Arbiter is the primary layer of abstraction for all supported Driver
 * instances, allowing remote paths to be treated as a simple filesystem or
 * key-value store.  Routing to specialized drivers is based on a substring
 * comparison of an incoming path's delimited type, for example
 * `http://arbiter.io/data.txt` would route to the HTTP driver and
 * `s3://my-bucket/data.txt` would route to the S3 driver.  Paths containing
 * no prefixed type information are routed to the filesystem driver.
 *
 * All Arbiter operations are thread-safe except unless otherwise noted.
 */
class ARBITER_DLL Arbiter
{
public:
    /** Construct a basic Arbiter with only drivers the don't require
     * external configuration parameters.
     */
    Arbiter();

    /** @brief Construct an Arbiter with driver configurations. */
    Arbiter(std::string stringifiedJson);

    /** True if a Driver has been registered for this file type. */
    bool hasDriver(std::string path) const;

    /** @brief Add a custom driver for the supplied type.
     *
     * After this operation completes, future requests into arbiter beginning
     * with the prefix @p type followed by the delimiter `://` will be routed
     * to the supplied @p driver.  If a Driver of type @p type already exists,
     * the supplied @p driver will replace it.
     *
     * This operation will throw ArbiterError if @p driver is empty.
     *
     * @note This operation is not thread-safe.
     */
    void addDriver(std::string type, std::unique_ptr<Driver> driver);

    /** Get data or throw if inaccessible. */
    std::string get(std::string path) const;

    /** Get data if accessible. */
    std::unique_ptr<std::string> tryGet(std::string path) const;

    /** Get data in binary form or throw if inaccessible. */
    std::vector<char> getBinary(std::string path) const;

    /** Get data in binary form if accessible. */
    std::unique_ptr<std::vector<char>> tryGetBinary(std::string path) const;

    /** Get file size in bytes or throw if inaccessible. */
    std::size_t getSize(std::string path) const;

    /** Get file size in bytes if accessible. */
    std::unique_ptr<std::size_t> tryGetSize(std::string path) const;

    /** Write data to path. */
    void put(std::string path, const std::string& data) const;

    /** Write data to path. */
    void put(std::string path, const std::vector<char>& data) const;

    /** Get data with additional HTTP-specific parameters.  Throws if
     * isHttpDerived is false for this path. */
    std::string get(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Get data with additional HTTP-specific parameters.  Throws if
     * isHttpDerived is false for this path. */
    std::unique_ptr<std::string> tryGet(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Get data in binary form with additional HTTP-specific parameters.
     * Throws if isHttpDerived is false for this path. */
    std::vector<char> getBinary(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Get data in binary form with additional HTTP-specific parameters.
     * Throws if isHttpDerived is false for this path. */
    std::unique_ptr<std::vector<char>> tryGetBinary(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Write data to path with additional HTTP-specific parameters.
     * Throws if isHttpDerived is false for this path. */
    void put(
            std::string path,
            const std::string& data,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Write data to path with additional HTTP-specific parameters.
     * Throws if isHttpDerived is false for this path. */
    void put(
            std::string path,
            const std::vector<char>& data,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Copy data from @p src to @p dst.  @p src will be resolved with
     * Arbiter::resolve prior to the copy, so globbed directories are supported.
     * If @p src ends with a slash, it will be resolved with a recursive glob,
     * in which case any nested directory structure will be recreated in @p dst.
     *
     * If @p dst is a filesystem path, mkdirp will be called prior to the
     * start of copying.  If @p src is a recursive glob, `mkdirp` will
     * be repeatedly called during copying to ensure that any nested directories
     * are reproduced.
     */
    void copy(std::string src, std::string dst, bool verbose = false) const;

    /** Copy the single file @p file to the destination @p to.  If @p to ends
     * with a `/` or '\' character, then @p file will be copied into the
     * directory @p to with the basename of @p file.  If @p to does not end
     * with a slash character, then @p to will be interpreted as a file path.
     *
     * If @p to is a local filesystem path, then `mkdirp` will be called
     * prior to copying.
     */
    void copyFile(std::string file, std::string to, bool verbose = false) const;

    /** Returns true if this path is a remote path, or false if it is on the
     * local filesystem.
     */
    bool isRemote(std::string path) const;

    /** Returns true if this path is on the local filesystem, or false if it is
     * remote.
     */
    bool isLocal(std::string path) const;

    /** Returns true if this path exists.  Equivalent to:
     * @code
     * tryGetSize(path).get() != nullptr
     * @endcode
     *
     * @note This means that an existing file of size zero will return true.
     */
    bool exists(std::string path) const;

    /** Returns true if the protocol for this driver is build on HTTP, like the
     * S3 and Dropbox drivers are.  If this returns true, http::Headers and
     * http::Query parameter methods may be used for this path.
     */
    bool isHttpDerived(std::string path) const;

    /** @brief Resolve a possibly globbed path.
     *
     * If @p path ends with `*`, then this operation will attempt to
     * non-recursively glob all files matching the given wildcard.  Directories
     * matching the wildcard and their contents are not returned.
     *
     * Examples:
     * @code
     * // Returns all files in `~/data`.  A directory `~/data/dir/` and its
     * // contents would not be returned since this is non-recursive.
     * a.resolve("~\data\*");
     *
     * // Returns all files matching `prefix-*` in bucket `my-bucket`.  An
     * // object `s3://my-bucket/prefix-123/hello.txt` would not be returned,
     * // but object `s3://my-bucket/prefix-456.txt` would.
     * a.resolve("s3://my-bucket/prefix-*");
     * @endcode
     *
     * @note Throws ArbiterError if the selected driver does not support
     * globbing, for example the HTTP driver.
     *
     * @param path Path to search, may be a directory or file.  Globbing
     * will only occur in the case of a directory.
     *
     * @param verbose If true, driver-specific status information may be
     * printed to STDOUT during globbing.
     *
     * @return  If @p path ends with `*`, the results are the contents of the
     * resulting non-recursive resolution of this path.  Otherwise, the results
     * are a vector of size one containing only @p path itself, unaltered.
     */
    std::vector<std::string> resolve(
            std::string path,
            bool verbose = false) const;

    /** @brief Get a reusable Endpoint for this root directory. */
    Endpoint getEndpoint(std::string root) const;

    /** Returns the Driver, if one can be found, for the given @p path.  The
     * driver type is determined from the @p path substring prior to the
     * delimiter `://`.  If this delimiter does not exist, then the filesystem
     * driver is returned.  If the delimiter exists but a corresponding driver
     * type is not found, ArbiterError is thrown.
     *
     * Optionally, filesystem paths may be explicitly prefixed with `file://`.
     */
    const Driver& getDriver(std::string path) const;

    /** @brief Get a LocalHandle to a possibly remote file.
     *
     * If @p path is remote (see Arbiter::isRemote), this operation will fetch
     * the file contents and write them to the local filesystem in the
     * directory represented by @p tmpEndpoint.  The contents of @p path are
     * not copied if @p path is already local.
     *
     * There are no filename guarantees if @p path is remote.  Use
     * LocalHandle::localPath to determine this.
     *
     * @note This operation will throw an ArbiterError if the path is remote
     * and @p tmpEndpoint is also remote.
     *
     * @param path Possibly remote path to fetch.
     *
     * @param tempEndpoint If @path is remote, the local copy will be created
     * at this Endpoint.
     *
     * @return A LocalHandle for local access to the resulting file.
     */
    LocalHandle getLocalHandle(
            std::string path,
            const Endpoint& tempEndpoint) const;

    /** @brief Get a LocalHandle to a possibly remote file.
     *
     * If @p tempPath is not specified, the environment will be searched for a
     * temporary location.
     */
    LocalHandle getLocalHandle(
            std::string path,
            std::string tempPath = "") const;

    /** @brief Get a LocalHandle to a possibly remote file.
     *
     * If @p tempPath is not specified, the environment will be searched for a
     * temporary location.
     */
    LocalHandle getLocalHandle(
            std::string path,
            http::Headers headers,
            http::Query query = http::Query()) const;

    /** Fetch the common HTTP pool, which may be useful when dynamically
     * constructing adding a Driver via Arbiter::addDriver.
     */
    http::Pool& httpPool() { return *m_pool; }

private:
    const drivers::Http* tryGetHttpDriver(std::string path) const;
    const drivers::Http& getHttpDriver(std::string path) const;

    DriverMap m_drivers;
    std::unique_ptr<http::Pool> m_pool;
};

} // namespace arbiter

#ifdef ARBITER_CUSTOM_NAMESPACE
}
#endif

// //////////////////////////////////////////////////////////////////////
// End of content of file: arbiter/arbiter.hpp
// //////////////////////////////////////////////////////////////////////





