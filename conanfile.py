#!/usr/bin/env python
# -*- coding: utf-8 -*-

from conan import ConanFile
from conan.tools.system.package_manager import Apt, PacMan
from conan.tools.files import get, copy
from conan.tools.cmake import CMake, CMakeToolchain
from conan.tools.env import VirtualBuildEnv
from conan.errors import ConanInvalidConfiguration
import json, os

required_conan_version = ">=2.0"

class PicoSecureGpioMirrorConan(ConanFile):

    jsonInfo = json.load(open("info.json", 'r'))
    # ---Package reference---
    name = jsonInfo["projectName"]
    version = jsonInfo["version"]
    user = jsonInfo["domain"]
    channel = "stable"
    # ---Metadata---
    description = jsonInfo["projectDescription"]
    license = jsonInfo["license"]
    author = jsonInfo["vendor"]
    topics = jsonInfo["topics"]
    homepage = jsonInfo["homepage"]
    url = jsonInfo["repository"]
    # ---Requirements---
    requires = []
    tool_requires = ["cmake/[>=3.13 <3.27]", "ninja/[>=1.11.1]"]
    # ---Sources---
    exports = ["info.json", "LICENSE"]
    exports_sources = ["info.json", "LICENSE", "*.txt", "src/*"]
    # ---Binary model---
    settings = "os", "compiler", "build_type", "arch"
    options = {"board": ["pico", "pico2"]}
    default_options = {"board": "pico"}
    # ---Build---
    generators = []
    # ---Folders---
    no_copy_source = True

    def validate(self):
        valid_os = ["baremetal"]
        if str(self.settings.os) not in valid_os:
            raise ConanInvalidConfiguration(
                f"{self.name} {self.version} is only supported for the following operating systems: {valid_os}")
        valid_arch = ["armv6", "armv7"]
        if str(self.settings.arch) not in valid_arch:
            raise ConanInvalidConfiguration(
                f"{self.name} {self.version} is only supported for the following architectures on {self.settings.os}: {valid_arch}")

    def build_requirements(self):
        self.tool_requires("raspberry-pi-pico-toolchain/2.2.0@de.privatehive/stable", options={"board": self.options.board})

    def generate(self):
        ms = VirtualBuildEnv(self)
        tc = CMakeToolchain(self, generator="Ninja")
        tc.generate()
        ms.generate()

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()
        cmake.install()

    def package(self):
        copy(self, pattern="gpio-mirror.*", src=os.path.join(self.build_folder, "src"), dst=self.package_folder)

    def deploy(self):
        copy(self, "gpio-mirror.uf2", src=self.package_folder, dst=self.deploy_folder)
