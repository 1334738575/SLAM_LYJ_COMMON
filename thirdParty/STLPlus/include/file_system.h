#ifndef STLPLUS_FILE_SYSTEM
#define STLPLUS_FILE_SYSTEM
////////////////////////////////////////////////////////////////////////////////

//   Author:    Andy Rushton
//   Copyright: (c) Southampton University 1999-2004
//              (c) Andy Rushton           2004 onwards
//   License:   BSD License, see ../docs/license.html

//   Simplified access to the File system

//   All file system access and filename manipulation should be done
//   with this package. Then it is only necessary to port this package
//   to port all file handling.

////////////////////////////////////////////////////////////////////////////////
#include "portability_fixes.h"
#include <string>
#include <vector>
#include <time.h>
////////////////////////////////////////////////////////////////////////////////

namespace stlplus
{

  ////////////////////////////////////////////////////////////////////////////////
  // implement string comparison of paths - Unix is case-sensitive, Windows is case-insensitive

    SLAM_LYJ_API bool path_compare(const std::string& l, const std::string& r);

  ////////////////////////////////////////////////////////////////////////////////
  // classifying functions

  // test for whether there's something (i.e. folder or file) with this name
    SLAM_LYJ_API bool is_present(const std::string& thing);
  // test for whether there's something present and its a folder
    SLAM_LYJ_API bool is_folder(const std::string& thing);
  // test for whether there's something present and its a file
  // a file can be a regular file, a symbolic link, a FIFO or a socket, but not a device
    SLAM_LYJ_API bool is_file(const std::string& thing);

  ////////////////////////////////////////////////////////////////////////////////
  // file functions

  // tests whether there's a file of this name
    SLAM_LYJ_API bool file_exists(const std::string& filespec);
  // tests whether the file is readable - i.e. exists and has read mode set
    SLAM_LYJ_API bool file_readable(const std::string& filespec);
  // tests whether file is writable - either it exists and is writable or doesn't exist but is in a writable folder
    SLAM_LYJ_API bool file_writable(const std::string& filespec);
  // the size of the file in bytes - 0 if doesn't exist
    SLAM_LYJ_API size_t file_size(const std::string& filespec);
  // delete the file - returns true if the delete succeeded
    SLAM_LYJ_API bool file_delete(const std::string& filespec);
  // rename the file - returns true if the rename succeeded
    SLAM_LYJ_API bool file_rename (const std::string& old_filespec, const std::string& new_filespec);
  // make an exact copy of the file - returns true if it succeeded
    SLAM_LYJ_API bool file_copy (const std::string& old_filespec, const std::string& new_filespec);
  // move the file - tries to rename, if that fails, tries to copy - returns true if either of these succeeded
    SLAM_LYJ_API bool file_move (const std::string& old_filespec, const std::string& new_filespec);

  // get the file's time stamps as a time_t - see the stlplus time.hpp package

  // time the file was originally created
    SLAM_LYJ_API time_t file_created(const std::string& filespec);
  // time the file was last modified
    SLAM_LYJ_API time_t file_modified(const std::string& filespec);
  // time the file was accessed
    SLAM_LYJ_API time_t file_accessed(const std::string& filespec);

  // platform-specific string handling to combine a directory and filename into a path

  // combine a folder with a filename (basename.extension)
    SLAM_LYJ_API std::string create_filespec(const std::string& folder, const std::string& filename);
  // combine a folder, a basename and an extension - extension does not need the .
    SLAM_LYJ_API std::string create_filespec(const std::string& folder, const std::string& basename, const std::string& extension);
  // combine a basename and an extension - extension does not need the .
    SLAM_LYJ_API std::string create_filename(const std::string& basename, const std::string& extension);

  ////////////////////////////////////////////////////////////////////////////////
  // folder functions

  // craete a folder - returns true if successful
    SLAM_LYJ_API bool folder_create(const std::string& folder);
  // tests for whether the folder exists, i.e. there is something of that name and its a folder
    SLAM_LYJ_API bool folder_exists(const std::string& folder);
  // test whether the folder contents are readable
    SLAM_LYJ_API bool folder_readable(const std::string& folder);
  // tests whether the folder can be written to - for example to create a new file
    SLAM_LYJ_API bool folder_writable(const std::string& folder);
  // delete the folder, optionally deleting the contents first - only succeeds if everything could be deleted
    SLAM_LYJ_API bool folder_delete(const std::string& folder, bool recurse = false);
  // rename the folder - this probably only works within a disk/partition
    SLAM_LYJ_API bool folder_rename (const std::string& old_directory, const std::string& new_directory);
  // test whether the folder is empty (of files)
    SLAM_LYJ_API bool folder_empty(const std::string& folder);

  // set the current folder
    SLAM_LYJ_API bool folder_set_current(const std::string& folder);

  // platform-specific string handling to retrieve some special locations
  // these do not check whether the folder exists, they just process strings

  // get the current folder
    SLAM_LYJ_API std::string folder_current(void);
  // get the current folder as a full path
    SLAM_LYJ_API std::string folder_current_full(void);
  // get the home folder - $HOME or %HOMEDRIVE%%HOMEPATH%
    SLAM_LYJ_API std::string folder_home(void);
  // go down a level in the folder hierarchy
    SLAM_LYJ_API std::string folder_down(const std::string& folder, const std::string& subfolder);
  // go up a level in the folder hierarchy
    SLAM_LYJ_API std::string folder_up(const std::string& folder, unsigned levels = 1);

  // get folder contents

  // the set of all subdirectories
    SLAM_LYJ_API std::vector<std::string> folder_subdirectories(const std::string& folder);
  // the set of all files
    SLAM_LYJ_API std::vector<std::string> folder_files(const std::string& folder);
  // the set of all folders and files
    SLAM_LYJ_API std::vector<std::string> folder_all(const std::string& folder);
  // the set of all folder contents matching a wildcard string
  // if folders is true, include folders; if files is true, include files
    SLAM_LYJ_API std::vector<std::string> folder_wildcard(const std::string& folder,
                                           const std::string& wildcard,
                                           bool folders = true,
                                           bool files = true);

  ////////////////////////////////////////////////////////////////////////////////
  // path functions

  // string manipulations of paths

  // test whether a string represents a full path or a relative one
  SLAM_LYJ_API bool is_full_path(const std::string& path);
  SLAM_LYJ_API bool is_relative_path(const std::string& path);

  // convert to a full path relative to the root path
  SLAM_LYJ_API std::string folder_to_path(const std::string& root, const std::string& folder);
  SLAM_LYJ_API std::string filespec_to_path(const std::string& root, const std::string& filespec);

  // convert to a full path relative to the current working directory
  SLAM_LYJ_API std::string folder_to_path(const std::string& folder);
  SLAM_LYJ_API std::string filespec_to_path(const std::string& filespec);

  // convert to a relative path relative to the root path
  SLAM_LYJ_API std::string folder_to_relative_path(const std::string& root, const std::string& folder);
  SLAM_LYJ_API std::string filespec_to_relative_path(const std::string& root, const std::string& filespec);

  // convert to a relative path relative to the current working directory
  SLAM_LYJ_API std::string folder_to_relative_path(const std::string& folder);
  SLAM_LYJ_API std::string filespec_to_relative_path(const std::string& filespec);

  // append a folder separator to the path to make it absolutely clear that it is a folder
  SLAM_LYJ_API std::string folder_append_separator(const std::string& folder);
  // undo the above to give a simplified folder with no trailing separator
  SLAM_LYJ_API std::string folder_remove_end_separator(const std::string& folder);

  ////////////////////////////////////////////////////////////////////////////////
  // access functions split a filespec into its elements

  // get the basename - that is, the name of the file without folder or extension parts
  SLAM_LYJ_API std::string basename_part(const std::string& filespec);
  // get the filename - that is, the name of the file without folder part but with extension
  SLAM_LYJ_API std::string filename_part(const std::string& filespec);
  // get the extension - that is the part of the filename after the . (and excluding the .)
  SLAM_LYJ_API std::string extension_part(const std::string& filespec);
  // get the folder part - that is the filespec with the filename removed
  SLAM_LYJ_API std::string folder_part(const std::string& filespec);

  // split a path into a vector of elements - i.e. split at the folder separator
  SLAM_LYJ_API std::vector<std::string> folder_elements(const std::string& folder);
  SLAM_LYJ_API std::vector<std::string> filespec_elements(const std::string& filespec);

  ////////////////////////////////////////////////////////////////////////////////
  // Path lookup functions

#ifdef MSWINDOWS
#define PATH_SPLITTER ";"
#else
#define PATH_SPLITTER ":"
#endif

  // The lookup normally carried out by the shell to find a command in a
  // directory in the PATH. Give this function the name of a command and it
  // will return the full path. It returns an empty string on failure.
  SLAM_LYJ_API std::string path_lookup (const std::string& command);

  // Generalised form of the above, takes a second argument
  // - the list to search. This can be used to do other path lookups,
  // such as LD_LIBRARY_PATH. The third argument specifies the splitter -
  // the default value of PATH_SPLITTER is appropriate for environment variables.
  SLAM_LYJ_API std::string lookup (const std::string& file, const std::string& path, const std::string& splitter = PATH_SPLITTER);

  // utility function for finding the folder that contains the current executable
  // the argument is the argv[0] parameter passed to main
  SLAM_LYJ_API std::string install_path(const std::string& argv0);

  ////////////////////////////////////////////////////////////////////////////////

} // end namespace stlplus

#endif
