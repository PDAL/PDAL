import os
import re
from conans import load, tools, ConanFile, CMake


def get_version():
    try:
        version = os.getenv('PROJECT_VERSION', None)
        if version:
            return version

        content = load('CMakeLists.txt')
        version = re.search('set\(PROJECT_VERSION (.*)\)', content).group(1)
        return version.strip()
    except:
        return None

class JsonSchemaValidatorConan(ConanFile):
    name = 'JsonSchemaValidator'
    version = get_version()
    url = 'https://github.com/pboettch/json-schema-validator'
    license = 'MIT'
    settings = 'os', 'compiler', 'build_type', 'arch'
    options = {
        'shared': [True, False],
        'fPIC': [True, False],
        'build_examples': [True, False],
        'build_tests': [True, False]
    }
    default_options = {
        'shared': False,
        'fPIC': True,
        'build_examples': True,
        'build_tests': False
    }
    generators = "CMakeDeps"
    exports_sources = [
        'CMakeLists.txt',
        'nlohmann_json_schema_validatorConfig.cmake.in',
        'src/*',
        'app/*',
        'test/*',
    ]
    requires = (
        'nlohmann_json/3.11.2'
    )
    _cmake = None

    def _configure_cmake(self):
        if self._cmake:
            return self._cmake
        self._cmake = CMake(self)
        self._cmake.definitions['JSON_VALIDATOR_BUILD_EXAMPLES'] = self.options.build_examples
        self._cmake.definitions['JSON_VALIDATOR_BUILD_TESTS'] = self.options.build_tests
        self._cmake.configure()
        return self._cmake

    def layout(self):
        build_type = str(self.settings.build_type).lower()
        self.folders.build = "build-{}".format(build_type)

    def build(self):
        cmake = self._configure_cmake()
        cmake.configure()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        includedir = os.path.join(self.package_folder, "include")
        self.cpp_info.includedirs = [includedir]

        libdir = os.path.join(self.package_folder, "lib")
        self.cpp_info.libdirs = [libdir]
        self.cpp_info.libs += tools.collect_libs(self, libdir)

        bindir = os.path.join(self.package_folder, "bin")
        self.output.info("Appending PATH environment variable: {}".format(bindir))
        self.env_info.PATH.append(bindir)

        self.user_info.VERSION = self.version
