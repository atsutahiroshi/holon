#!/bin/bash -eu

# This script builds cure, zm, zeo and roki libraries on Linux environment.

SCRIPT_DIR="$(cd "$(dirname "$0")" ; pwd -P)"   # /path/to/holon/third_party/linux
THIRD_PARTY_DIR="$(dirname "$SCRIPT_DIR")"      # /path/to/holon/third_party
TOP_DIR="$(dirname "$THIRD_PARTY_DIR")"         # /path/to/holon
PREFIX="$THIRD_PARTY_DIR/build"
BINDIR="$PREFIX/bin"
INCDIR="$PREFIX/include"
LIBDIR="$PREFIX/lib"
LIBS="cure zm zeo roki"


usage() {
  cat <<END;
Usage:
  ${0} [OPTION]...
Build CURE, ZM, Zeo and RoKi libraries on Linux environment.

Options:
      --prefix              Set PREFIX (Default: $PREFIX)
  -h, --help                Show this message.

END
}


check_args() {
  while [[ $# -gt 0 ]]; do
    case $1 in
      --prefix)
        PREFIX=$2
        shift 2
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      --)
        shift
        break
        ;;
      --*|-*)
        echo "unrecognized option: $1" >&2
        usage
        exit 1
        ;;
      *)
        echo "unrecognized argument: $1" >&2
        usage
        exit 1
        ;;
    esac
  done
}


##################################################
# Create config file
# Globals:
#   PREFIX: prefix path
# Arguments:
#   1: target library name
# Returns:
#   None
##################################################
create_config() {
  local libpath="$THIRD_PARTY_DIR/$1"
  sed -e "s@^PREFIX=.*\$@PREFIX=${PREFIX}@g" $libpath/config.org >$libpath/config
}


##################################################
# Prepare directories which are needed for installation
# Globals:
#   BINDIR: user executables
#   INCDIR: header files
#   LIBDIR: object code libraries
# Arguments:
#   None
# Returns:
#   None
##################################################
prepare_dirs() {
  mkdir -p $BINDIR
  mkdir -p $INCDIR
  mkdir -p $LIBDIR
}


##################################################
# Execute 'make && make install' command at target library
# Globals:
#   None
# Arguments:
#   1: target library name
# Returns:
#   0: success
#   1: fail
##################################################
makeit() {
  local libname=$1
  local libpath=$THIRD_PARTY_DIR/$libname
  local ret=
  create_config $libname
  cd $libpath
  make && make install
  ret=$?
  cd - 1>/dev/null
  return $ret
}


##################################################
# Activate the environment by setting appropriate variabels
# Globals:
#   PREFIX: prefix path
# Arguments:
#   None
# Returns:
#   None
##################################################
activate() {
  export OLD_PATH=$PATH
  export OLD_LD_LIBRARY_PATH=$LD_LIBRARY_PATH
  export PATH=$PREFIX/bin:$PATH
  export LD_LIBRARY_PATH=$PREFIX/lib:$LD_LIBRARY_PATH
}


##################################################
# Dectivate the environment
# Globals:
#   None
# Arguments:
#   None
# Returns:
#   None
##################################################
deactivate() {
  export PATH=$OLD_PATH
  export LD_LIBRARY_PATH=$OLD_LD_LIBRARY_PATH
  unset OLD_PATH
  unset OLD_LD_LIBRARY_PATH
}


##################################################
# Export activation script
# Globals:
#   None
# Arguments:
#   None
# Returns:
#   None
##################################################
export_activate() {
  cat <<END >$PREFIX/bin/activate
# This file must be used wiht "source bin/activate" from bash
# you cannot run it directly

deactivate() {
  export PATH=\$OLD_PATH
  export LD_LIBRARY_PATH=\$OLD_LD_LIBRARY_PATH
  unset OLD_PATH
  unset OLD_LD_LIBRARY_PATH
  unset -f deactivate
}

export OLD_PATH=\$PATH
export OLD_LD_LIBRARY_PATH=\$LD_LIBRARY_PATH
export PATH=$PREFIX/bin:\$PATH
export LD_LIBRARY_PATH=$PREFIX/lib:\$LD_LIBRARY_PATH
END
}


##################################################
# Show instruction message
# Globals:
#   None
# Arguments:
#   None
# Returns:
#   None
##################################################
show_instruction() {
  cat <<END


*********************************************************
Please activate the environmet by typing the following:
  \$ source $PREFIX/bin/activate
*********************************************************
END
}

# main
check_args "$@"
activate
prepare_dirs
for lib in $LIBS; do
  makeit $lib
done
deactivate
export_activate
show_instruction
