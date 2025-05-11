/******************************************************************************
 * Copyright (c) 2022, Kyle Mann (kyle@hobu.co)
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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
#include "Collection.hpp"

#include "Utils.hpp"
#include <nlohmann/json.hpp>
#include <schema-validator/json-schema.hpp>

namespace pdal
{

namespace stac
{

using namespace StacUtils;

Collection::~Collection() {}

void Collection::validate() {
    nlohmann::json_schema::json_validator val(
        [this](const nlohmann::json_uri& json_uri, nlohmann::json& json) {
            json = m_connector.getJson(json_uri.url());
        },
        [](const std::string &, const std::string &) {}
    );

    // Validate against base Collection schema first
    nlohmann::json schemaJson = m_connector.getJson(m_schemaUrls.collection);
    val.set_root_schema(schemaJson);
    try {
        val.validate(m_json);
    }
    catch (std::exception &e)
    {
        throw stac_error(m_id, "collection",
            "STAC schema validation Error in root schema: " +
            m_schemaUrls.collection + ". \n\n" + e.what());
    }

    // Validate against stac extensions if present
    if (m_json.contains("stac_extensions"))
    {
        nlohmann::json extensions = stacValue(m_json, "stac_extensions");
        for (auto& extSchemaUrl: extensions)
        {
            std::string url = stacValue<std::string>(extSchemaUrl,
                "", m_json);

            try {
                nlohmann::json schemaJson = m_connector.getJson(url);
                val.set_root_schema(schemaJson);
                val.validate(m_json);
            }
            catch (std::exception& e) {
                std::string msg  =
                    "STAC Validation Error in extension: " + url +
                    ". Errors found: \n" + e.what();
                throw stac_error(m_id, "collection", msg);

            }
        }

    }
}


}//stac

}//pdal