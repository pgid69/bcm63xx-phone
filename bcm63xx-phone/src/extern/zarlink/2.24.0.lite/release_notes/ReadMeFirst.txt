Microsemi Corporation VP-API-II (LE71SDKAPIL), version P2.24.0

Release Notes
Nov 17 2014
Copyright (c) 2014 Microsemi Corporation

Descriptions of the files included in this release are below:
=============================================================
NOTES:  The selection of files installed in the following directories is 
        dependent on the device family selection made during the installation 
        process.

Install Folder\release_notes\
        Contains the "Release and Errata" notes for this release.

Install Folder\documents\
        Contains VP-API documentation. Please review the necessary documentation
        carefully before making use of VP-API.

Install Folder\api_lib\
        VP-API Library.

Install Folder\apps\
        Applications built on VP-API. These applications provide the basic
        steps necessary to develop an application using the VP-API-II.

Install Folder\coefficient_sets\VE880 (if VE880 is included in installation)
        Contains approved coefficients for VE880 reference designs.

Install Folder\coefficient_sets\VE890 (if VE890 is included in installation)
        Contains approved coefficients for VE890 reference designs.

Install Folder\coefficient_sets\ZL880 (if ZL880 is included in installation)
        Contains approved coefficients for ZL880 reference designs.

Install Folder\arch\
        Contains example implementations of Hardware Abstraction Layer and 
        System Services layer.

api_lib directory
=================
Install Folder\api_lib\includes\
        Contains the include files necessary for importing VP-API library. It 
        is typically sufficient if applications include vp_api.h in their 
        programs. This file brings all the necessary definitions for making use 
        of VP_API. It is sufficient to have an include path for just this 
        directory.

Install Folder\api_lib\includes\vp_api_cfg.h
        This is the most important file the users will have to carefully review
        and configure. This file determines various compile time options that 
        the VP-API makes use of.

        The support for libraries is also determined in this file. By default
        installation comes with all libraries DISABLED. Users should enable the
        necessary libraries by defining necessary conditional flags.
       
Install Folder\api_lib\common\
        Includes necessary files to support common functions of the VP-API.

Install Folder\api_lib\vp886_api\
        Includes necessary files to support ZL880 series devices.

Install Folder\api_lib\vp890_api\
        Includes necessary files to support VE890 series devices.

Install Folder\api_lib\vp880_api\
        Includes necessary files to support VE880 series devices.

Install Folder\api_lib\vp790_api\
        Includes necessary files to support VE790 series devices.

Install Folder\api_lib\vp580_api\
        Includes necessary files to support VE580 series devices.

apps directory
==============
        This directory contains applications that are tested on Microsemi Voice 
        platforms. Users should be able to port these applications to their 
        platforms with minimal modifications. Typically the directories contain
        documentation that describes the application.

arch directory
==============
        This directory contains example implementations of Hardware 
        Abstraction Layer (HAL) and System Services Layer (SS) implementation. It
        also contains skeleton files to be filled in by the customer.

        Users of VP-API will need to develop HAL and SS layers as applicable to
        to the hardware and software environments they are working.


All technical details of this release described in the "Release  and Errata Notice"
found in directory "install_dir\release_notes".