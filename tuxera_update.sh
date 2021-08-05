#!/bin/bash
# vim: ft=bash sw=4 sts=4 et

# ================= NO CHANGE NEEDED BELOW! =====================

usage() {
    usagestr=$(cat <<EOF
Usage: tuxera_update.sh [OPTION...]

  This script only assembles the kernel headers package
  (kheaders.tar.bz2) by default. Use -a to invoke Autobuild.

  --output-dir OUTDIR

      Specify kernel build output directory.

  --no-excludes

      Do not exclude *.c, *.o, *.S, arch/*/boot. Only use if the build
      fails (some of the excluded files are needed). This significantly
      grows the headers package size.

  --user USER [--pass PASSWD] [--admin ADMIN]
      If remote connectivity is needed (-a or --latest), username and
      password are required. If missing, they will be read from stdin.

      NOTE: Using --pass can be dangerous as local users see it in 'ps'.

      The --admin option lets you log in using the password of the given
      ADMIN user.

  --use-cache [--cache-dir CACHEDIR] [--latest] [--max-cache-entries N]
      Obtain modules from local cache if kernel dependencies are not
      modified. You must provide --output-dir to
      use the cache. Modules are relinked on cache hit, ensure the
      toolchain is in PATH. If -a is specified, Autobuild is invoked
      on a cache miss, and the cache is updated. Otherwise, kernel headers
      assembly is performed for later, manual build.

      You can specify a different cache directory with --cache-dir. The
      default directory is \$PWD/.tuxera_update_cache

      The --latest option will ensure that the release found in cache is
      the latest version available on Autobuild servers. This needs remote
      connectivity. If a new version is found and -a is specified,
      Autobuild is invoked and the cache is updated.

      The optional --max-cache-entries can be used to limit the maximum
      cache size to N recently used entries. The default is 10.

  -a [--target TARGET] [OPTIONS]

      Start Autobuild against TARGET. If target was not specified, uses
      the default target. Use '--target list' to show available targets.

      The following extra options are supported:

        --use-package PACKAGE
          Autobuild starts with fresh .tar.bz2 assembly by default, but
          you can use this option to start Autobuild by uploading a pre-built
          .tar.bz2 file PACKAGE.

        --ignore-cert
          If up/download fails due to certificate issues, this option can
          be used to disable verification.

        --use-curl
        --use-wget
          Force the use of 'curl' or 'wget' for remote communication.

  -u, --upgrade

      Upgrade online to the latest script version, obtained from
      $upgrade_url

  -h, --help

      Prints this help.

  -v

      Only print the version of this script and exit.

  --verbose

      Print more debug information.

  Tuxera Autobuild Team
  autobuild-support@tuxera.com
EOF
)

  if [ -n "$long_help" ] ; then
      echo "$usagestr"
      echo
  else
      echo "For long help, use -h. Options:"
      echo "$usagestr" | grep "^ *-"
  fi
  exit 1
}

archs="alpha arc arm arm64 avr32 blackfin c6x cris frv h8300 hexagon ia64 m32r \
    m68k metag microblaze mips mn10300 openrisc parisc ppc s390 score \
    superh sparc tile unicore32 x86 xtensa ubicom32 csky"

#
# Detect required architecture directories based on autoconf.h
#
detect_arch_dirs() {
    # Simplify conditionals, if output_dir not set
    dir1="$source_dir"
    [ -n "$output_dir" ] && dir2="$output_dir" || dir2="$dir1"

    # autoconf.h must be present
    ac="${dir2}/include/generated/autoconf.h"
    [ -e "$ac" ] || ac="${dir2}/include/linux/autoconf.h"
    [ -e "$ac" ] || ac="${dir1}/include/generated/autoconf.h"
    [ -e "$ac" ] || ac="${dir1}/include/linux/autoconf.h"
    [ -e "$ac" ] || { echo "ERROR: Unable to detect kernel architecture: autoconf.h not found."; return 1; }

    pattern=""
    # Compute grep pattern
    for arch in $archs; do
        [ -n "$pattern" ] && separ="\|" || separ=""
        pattern="${pattern}${separ}^#define CONFIG_$(echo $arch | tr '[:lower:]' '[:upper:]')[[:space:]]"
    done

    # Find architecture from autoconf.h
    def=$(grep -e "$pattern" "$ac") || { echo "ERROR: Unable to detect kernel architecture: no macro found in autoconf.h."; return 1; }
    archdirs=$(echo "$def" | head -n 1 | sed -n 's/#define CONFIG_\(.*\)[[:space:]].*/\1/p' | tr '[:upper:]' '[:lower:]')

    # Fix some architecture names to get the correct directory
    [ "$archdirs" == "ppc" ] && archdirs="powerpc"
    [ "$archdirs" == "superh" ] && archdirs="sh"

    if [ "$archdirs" == "x86" ] ; then
        # Older kernels don't have x86 dir at all.
        # Some x86 builds require 2 directories. This is very rare.
        grep -q '^#define CONFIG_X86_32' "$ac" && archdirs="$archdirs i386"
        grep -q '^#define CONFIG_X86_64' "$ac" && archdirs="$archdirs x86_64"
    fi

    if [ "$archdirs" == "arm64" ] ; then
        # Nvidia Tegra 3.10.61
        grep -q '^#define CONFIG_ARCH_TEGRA' "$ac" && archdirs="$archdirs arm"
    fi

    # Realtek RLX fix
    [ "$archdirs" == "mips" ] && grep -q '^#define CONFIG_CPU_RLX' "$ac" && archdirs="rlx"

    # Realtek mips-ori fix
    [ "$archdirs" == "mips" ] && grep -q 'Linux/mips-ori' "$ac" && archdirs="$archdirs mips-ori"

    # Only select existing directories.
    existing=''
    for d in $archdirs ; do
        [ -e "$dir1/arch/$d" ] || [ -e "$dir2/arch/$d" ] && existing="$existing $d"
    done

    # Don't continue if there is no arch directory.
    [ -z "$existing" ] && { echo "ERROR: No arch directories found. Cannot continue."; return 1; }
    archdirs="$existing"
    echo "Kernel architecture directories: $archdirs"
}

#
# Check KH against typical errors
#
check_kh() {
    source_link="$1"
    output_link="$2"

    [ -e "${output_link}" ] || output_link="${source_link}"

    if ! [ -e "${source_link}/Makefile" ]; then
        echo "  ERROR: Kernel source code directory is invalid (no Makefile found).";
        echo "         To fix it, set the --output-dir parameter correctly.";
        return 1
    fi

    if grep -q "VERSION" ${source_link}/Makefile &&
       grep -q "PATCHLEVEL" ${source_link}/Makefile &&
       grep -q "SUBLEVEL" ${source_link}/Makefile
    then
        echo "Found valid Linux kernel Makefile at ${source_link}/Makefile"
    else
        echo "  ERROR: Kernel source code directory is invalid.";
        echo "         Makefile doesn't have VERSION, PATCHLEVEL and SUBLEVEL."
        echo "         To fix it, set the --output-dir parameter correctly.";
        return 1
    fi

    if ! [ -e "${output_link}/Module.symvers" ]; then
        echo "  ERROR: Invalid kernel configuration:";
        echo "         Module.symvers is missing.";
        echo "         Either the kernel was not compiled yet, or the kernel output directory is not correct.";
        echo "         Please make sure the kernel has been compiled, then use that directory for the --output-dir";
        echo "         argument where the Module.symvers file can be found."
        return 1
    fi

    [ -n "${disable_kernel_headers_check}" ] && return 0

    error_config_modules="  ERROR: Update kernel configuration:
         Please enable loadable modules support in kernel: CONFIG_MODULES=y
         (Enable loadable module support (CONFIG_MODULES)[Y/n/?] Y)
         Then rebuild the kernel and recollect kernel headers.

"
    error_config_cma="  ERROR: Fix kernel source code:
         CONFIG_CMA is enabled but 'is_cma_pageblock' is not exported!
         Please add EXPORT_SYMBOL(is_cma_pageblock) after the definition of this
         function (usually in mm/page_alloc.c).
         Then rebuild the kernel and recollect kernel headers.

"
    error_config_lockdep="  ERROR: Update kernel configuration:
         CONFIG_DEBUG_LOCK_ALLOC=y and/or CONFIG_LOCKDEP=y are enabled.
         These options convert some symbols required by file systems into
         GPL-only symbols, which legally cannot be used by our filesystem driver(s).
         Please disable these options, rebuild the kernel and recollect kernel headers.

"
    if ! [ -s "${output_link}/Module.symvers" ] &&
       ! grep -sxq 'CONFIG_MODULES=y' "${output_link}/.config"
    then
        errors+="${error_config_modules}"
        config_modules_not_set="yes"
    fi

    if grep -sxq 'CONFIG_CMA=y' "${output_link}/.config" &&
       grep -sq 'is_cma_pageblock' "${source_link}/include/linux/pagemap.h"
    then
        if [ -n "${config_modules_not_set}" ]; then
            errors+="${error_config_cma}"
        else
            grep -wq 'is_cma_pageblock' "${output_link}/Module.symvers" || errors+="${error_config_cma}"
        fi
    fi

    if grep -qx -e 'CONFIG_DEBUG_LOCK_ALLOC=y' -e 'CONFIG_LOCKDEP=y' "${output_link}/.config"
    then
       if [ -n "${config_modules_not_set}" ]; then
           errors+="${error_config_lockdep}"
       else
           awk '$4 == "EXPORT_SYMBOL_GPL" && $2 ~ /(mutex_lock_nested|lockdep_init_map|debug_check_no_locks_held)/ { exit(1); }' \
                   "${output_link}/Module.symvers" ||
               errors+="${error_config_lockdep}"
       fi
    fi

    [ -n "${errors}" ] && { printf "${errors}"; return 1; }

    return 0
}

#
# Assemble kernel headers package.
#
build_package() {
    if [ -z "$source_dir" ] ; then
        echo "You must specify --output-dir for headers package assembly."
        usage
    fi

    # Temporary directory that will contain links to kernel and output directories.
    LINK_DIR=$(mktemp -d)

    if [ $? -ne 0 ] ; then
        echo "mktemp failed. Unable to continue."
        exit 1
    fi

    KERNEL_LINK="${LINK_DIR}"/kernel
    OUTPUT_LINK="${LINK_DIR}"/output
    MEDIATEK_LINK="${LINK_DIR}"/mediatek
    MSTAR2_LINK="${LINK_DIR}"/mstar2
    EXTRA_SEARCH_NEW_MEDIATEK="${KERNEL_LINK}"/drivers/misc/mediatek/mach
    EXTRA_SEARCH_NEW_MEDIATEK_INCLUDE="${KERNEL_LINK}"/drivers/misc/mediatek/include
    EXTRA_SEARCH_NEW_MEDIATEK_BASE="${KERNEL_LINK}"/drivers/misc/mediatek/base
    EXTRA_SEARCH_AMLOGIC_DEBUG="${KERNEL_LINK}"/drivers/amlogic/debug
    [ -d "$source_dir"/../mediatek ] && have_mediatek=1
    [ -d "$(readlink -f "$source_dir"/../mstar2)" ] && have_mstar2=1
    [ -d "$(readlink -f "$source_dir"/mstar2)" ] && have_mstar2_inkernel=1

    # Create kernel and output links to LINK_DIR. Only create link to output
    # dir if output directory was specified. Note that absolute paths
    # must be used!
    ln -sf "$(readlink -f "$source_dir")" "${KERNEL_LINK}" && \
    if [ -n "$output_dir" ]; then ln -sf "$(readlink -f "$output_dir")" "${OUTPUT_LINK}"; fi && \
    if [ -n "$have_mediatek" ] ; then ln -sf "$(readlink -f "$source_dir")"/../mediatek "${MEDIATEK_LINK}"; fi

    if [ $? -ne 0 ] ; then
        echo "Symlinking (ln -s) failed. Unable to continue."
        rm -rf "${LINK_DIR}"
        exit 1
    fi

    if [ -e "${OUTPUT_LINK}" ] ; then dot_config="${OUTPUT_LINK}/.config"; else dot_config="${KERNEL_LINK}/.config"; fi
    if [ -n "$have_mstar2" ] ; then
        ln -sf "$(readlink -f "$source_dir")"/../mstar2 "${MSTAR2_LINK}"
        if [ -n "$have_mstar2_inkernel" ] ; then
            EXTRA_SEARCH_MSTAR2_INKERNEL="${KERNEL_LINK}"/mstar2/hal
        fi
    elif [ -n "$have_mstar2_inkernel" ] ; then
        ln -sf "$(readlink -f "$source_dir")"/mstar2 "${MSTAR2_LINK}"
    fi

    if [ -d "$(readlink -f "${MSTAR2_LINK}")" ] && grep -q 'CONFIG_MSTAR_CHIP' "$dot_config"; then
        MSTAR2_CHIP_NAME=$(awk -F= '/^CONFIG_MSTAR_CHIP_NAME=/{gsub(/[[:space:],\"]/,"",$2); print $2}' "$dot_config")
        [ ! -d "${MSTAR2_LINK}/hal/${MSTAR2_CHIP_NAME}" ] && MSTAR2_CHIP_NAME=""
    fi

    if [ "$source_dir" = "$output_dir" ] ; then
        rm "$OUTPUT_LINK"
    fi

    if ! check_kh "${KERNEL_LINK}" "${OUTPUT_LINK}"; then
        rm -rf "${LINK_DIR}"
        exit 1
    fi

    echo "Generating list of files to include..."
    INCLUDEFILE=$(mktemp)
    echo "${script_version}" >> "${LINK_DIR}"/.version
    echo "${LINK_DIR}"/.version >> "${INCLUDEFILE}"

    if [ $? -ne 0 ] ; then
        echo "mktemp failed. Unable to continue."
        rm -rf "${LINK_DIR}"
        exit 1
    fi

    if ! detect_arch_dirs; then
        rm -rf "${LINK_DIR}"
        exit 1
    fi

    SEARCHPATHS="$(for d in $archdirs; do echo "${KERNEL_LINK}/arch/$d"; done) ${KERNEL_LINK}/include ${KERNEL_LINK}/scripts ${KERNEL_LINK}/mediatek/Makefile ${KERNEL_LINK}/mediatek/platform ${MEDIATEK_LINK}/config ${MEDIATEK_LINK}/platform ${MEDIATEK_LINK}/build/libs ${MEDIATEK_LINK}/build/Makefile ${MEDIATEK_LINK}/build/kernel ${MEDIATEK_LINK}/kernel/include/linux ${MEDIATEK_LINK}/kernel/Makefile ${MSTAR2_LINK}/hal/${MSTAR2_CHIP_NAME} ${EXTRA_SEARCH_MSTAR2_INKERNEL} ${KERNEL_LINK}/KMC ${KERNEL_LINK}/init/secureboot ${KERNEL_LINK}/mvl-avb-version ${EXTRA_SEARCH_NEW_MEDIATEK} ${EXTRA_SEARCH_NEW_MEDIATEK_INCLUDE} ${EXTRA_SEARCH_NEW_MEDIATEK_BASE} ${EXTRA_SEARCH_AMLOGIC_DEBUG} ${KERNEL_LINK}/tools/gcc ${KERNEL_LINK}/tools/Makefile ${KERNEL_LINK}/tools/objtool ${KERNEL_LINK}/tools/scripts"

    if [ -L "${OUTPUT_LINK}" ] ; then
        SEARCHPATHS="${SEARCHPATHS} $(for d in $archdirs; do echo "${OUTPUT_LINK}/arch/$d"; done) ${OUTPUT_LINK}/include ${OUTPUT_LINK}/scripts ${OUTPUT_LINK}/KMC ${OUTPUT_LINK}/init/secureboot ${OUTPUT_LINK}/mvl-avb-version ${OUTPUT_LINK}/tools/gcc ${OUTPUT_LINK}/tools/Makefile ${OUTPUT_LINK}/tools/objtool ${OUTPUT_LINK}/tools/scripts"
    fi

    # Not all of the directories always exist, so check for them first to avoid an error message
    # from 'find' if the directory is not found.
    for P in ${SEARCHPATHS}; do
        if [ ! -e "${P}" ]; then continue; fi
        if [ -n "$no_excludes" ] ; then
            find -L "${P}" ! -type l >> "${INCLUDEFILE}"
        else
            find -L "${P}" \
            \( ! -type l -a ! -name \*.c -a ! -name \*.o -a ! -name \*.S -a ! -name \*.ko -a ! -name \*.dts -a ! -name \*.dtsi -a ! -path \*/scripts/dtc\* -a ! -path \*/arch/\*/boot/\* -a ! -path \*/.svn/\* -a ! -path \*/.git/\* ! -path \*mediatek/platform/Android.mk ! -path \*mediatek/platform/README ! -path \*mediatek/platform/common\* ! -path \*mediatek/platform/rules.mk ! -path \*mediatek/platform/\*/Trace\* ! -path \*mediatek/platform/\*/external\* ! -path \*mediatek/platform/\*/hardware\* ! -path \*mediatek/platform/\*/lk\* ! -path \*mediatek/platform/\*/preloader\* ! -path \*mediatek/platform/\*/uboot\* ! -path \*mediatek/platform/\*/kernel/core/Makefile ! -path \*mediatek/platform/\*/kernel/core/Makefile.boot  ! -path \*mediatek/platform/\*/kernel/core/modules.builtin ! -path \*mediatek/platform/\*/kernel/drivers\* ! -path \*mediatek/platform/\*/kernel/Kconfig\* ! -path \*mediatek/config/a830\* ! -path \*mediatek/config/banyan_addon\* ! -path \*mediatek/config/out\* ! -path \*mediatek/config/prada\* ! -path \*mediatek/config/s820\* ! -path \*mediatek/config/seine\* \) \
            >> ${INCLUDEFILE}
        fi
    done

    echo ${KERNEL_LINK}/Makefile >> ${INCLUDEFILE}
    if [ -e "${OUTPUT_LINK}/Makefile" ]; then echo "${OUTPUT_LINK}/Makefile" >> ${INCLUDEFILE}; fi
    if [ -e "${KERNEL_LINK}/Module.symvers" ]; then echo "${KERNEL_LINK}/Module.symvers" >> ${INCLUDEFILE}; fi
    if [ -e "${OUTPUT_LINK}/Module.symvers" ]; then echo "${OUTPUT_LINK}/Module.symvers" >> ${INCLUDEFILE}; fi
    if [ -e "${KERNEL_LINK}/.config" ]; then echo "${KERNEL_LINK}/.config" >> ${INCLUDEFILE}; fi
    if [ -e "${OUTPUT_LINK}/.config" ]; then echo "${OUTPUT_LINK}/.config" >> ${INCLUDEFILE}; fi
    if [ -e "${OUTPUT_LINK}/arch/powerpc/lib/crtsavres.o" ]; then echo "${OUTPUT_LINK}/arch/powerpc/lib/crtsavres.o" >> ${INCLUDEFILE}; fi
    if [ -e "${KERNEL_LINK}/arch/powerpc/lib/crtsavres.o" ]; then echo "${KERNEL_LINK}/arch/powerpc/lib/crtsavres.o" >> ${INCLUDEFILE}; fi
    if [ -e "${OUTPUT_LINK}/scripts/recordmcount.c" ]; then echo "${OUTPUT_LINK}/scripts/recordmcount.c" >> ${INCLUDEFILE}; fi
    if [ -e "${KERNEL_LINK}/scripts/recordmcount.c" ]; then echo "${KERNEL_LINK}/scripts/recordmcount.c" >> ${INCLUDEFILE}; fi
    if [ -e "${OUTPUT_LINK}/scripts/recordmcount.h" ]; then echo "${OUTPUT_LINK}/scripts/recordmcount.h" >> ${INCLUDEFILE}; fi
    if [ -e "${KERNEL_LINK}/scripts/recordmcount.h" ]; then echo "${KERNEL_LINK}/scripts/recordmcount.h" >> ${INCLUDEFILE}; fi

    # Commit 338d4f49d6f7114a017d294ccf7374df4f998edc requires arm64 some kernel headers from arm
    case "$archdirs" in *arm64*)
            arm64_need_arm_files="arch/arm/include/asm/opcodes.h arch/arm/include/asm/xen/hypervisor.h arch/arm/include/asm/xen/interface.h include/asm/xen/interface.h"
            for EF in ${arm64_need_arm_files}; do
                if [ -e "${OUTPUT_LINK}/${EF}" ]; then echo "${OUTPUT_LINK}/${EF}" >> ${INCLUDEFILE}; fi
                if [ -e "${KERNEL_LINK}/${EF}" ]; then echo "${KERNEL_LINK}/${EF}" >> ${INCLUDEFILE}; fi
            done
            ;;
    esac

    echo "Packing kernel headers ..."
    tar cjf "${1}" --dereference --no-recursion --files-from "${INCLUDEFILE}"

    if [ $? -ne 0 ] ; then
        echo "tar packaging failed. I will now exit."
        rm -rf "${LINK_DIR}"
        exit 1
    fi

    rm ${INCLUDEFILE}
    rm -rf "${LINK_DIR}"
    echo "Headers package assembly succeeded. You could now use --use-package ${1}."
}

#
# Retry network operations
#
do_retry() {
    local result
    local status
    local delay=$conn_fail_retry_delay
    local retr=1
    while :
    do
        result=$("$@")
        status=$?
        if [ $status -eq 0 ] || [ $retr -gt $conn_fail_retry ] || [ -z "$retry_on_conn_fail" ] ; then
            break
        fi
        echo "Trying to reconnect in "$delay" seconds..." >&2
        sleep $delay
        delay=$(($delay * 2))
        retr=$(($retr + 1))
    done
    [ -n "$result" ] && echo "$result"
    return $status
}

#
# Upload kernel headers package using curl.
#
upload_package_curl() {
    local retr=1
    local delay=$conn_fail_retry_delay
    local execution_status
    while :
    do
        execution_status=0
        echo "Uploading the following package:"
        ls -lh "${1}"

        reply=$(do_retry $curl -F "file=@${1}" https://${server}/upload.php)

        if [ $? -ne 0 ] ; then
            echo "curl failed. Unable to continue. Check connectivity and username/password." >&2
            execution_status=1
        fi

        if [ $execution_status -eq 0 ] ; then
            status=$(echo "$reply" | head -n 1)

            if [ "$status" != "OK" ] ; then
                echo "$reply" >&2
                echo "Upload failed. Unable to continue." >&2
                execution_status=1
            fi
        fi

        if [ $execution_status -ne 0 ] ; then
            if [ $retr -gt $conn_fail_retry ] || [ -z "$retry_on_conn_fail" ] ; then
                exit 1
            else
                echo "Will try one more time $retr of $conn_fail_retry in "$delay" seconds..." >&2
                sleep $delay
                retr=$(($retr + 1))
                delay=$(($delay * 2))
                continue
            fi

        fi

        remote_package=$(echo "$reply" | head -n 2 | tail -n 1)

        echo "Upload succeeded."
        break
    done
}

#
# Upload kernel headers package using wget.
# Package needs to be urlencoded.
#
upload_package_wget() {
    encoded="$(mktemp)"
    echo "urlencoding $1 to $encoded ..."

    echo -n "file=" > "$encoded"
    perl -pe 's/([^-_.~A-Za-z0-9])/sprintf("%%%02X", ord($1))/seg' < "$1" >> "$encoded"

    if [ $? -ne 0 ] ; then
        echo "urlencoding failed. Unable to continue."
        exit 1
    fi
    local retr=1
    local delay=$conn_fail_retry_delay
    local execution_status
    while :
    do
        execution_status=0
        echo "Uploading package..."

        reply=$(do_retry $wget --post-file="$encoded" -O - https://${server}/upload.php)

        if [ $? -ne 0 ] ; then
            echo "wget failed. Unable to continue. Check connectivity and username/password." >&2
            execution_status=1
        fi

        if [ $execution_status -eq 0 ] ; then
            status=$(echo "$reply" | head -n 1)

            if [ "$status" != "OK" ] ; then
                echo "$reply"
                echo "Upload failed. Unable to continue." >&2
                execution_status=1
            fi
        fi

        if [ $execution_status -ne 0 ] ; then
            if [ $retr -gt $conn_fail_retry ] || [ -z "$retry_on_conn_fail" ] ; then
                rm "$encoded"
                exit 1
            else
                echo "Will try one more time $retr of $conn_fail_retry in "$delay" seconds..." >&2
                sleep $delay
                retr=$(($retr + 1))
                delay=$(($delay * 2))
                continue
            fi

        fi

        remote_package=$(echo "$reply" | head -n 2 | tail -n 1)
        rm "$encoded"

        echo "Upload succeeded."
        break
    done
}

#
# Find headers that were used during dependency module compilation from
# the *.i files. Compute their checksums.
#
calc_header_checksums() {
    # Holds initial list of header files.
    DEPENDENCY=$(mktemp)

    if [ $? -ne 0 ] ; then
        echo "mktemp failed. Unable to continue."
        return 1
    fi

    # Find header files from the *.i files.
    egrep '^#.*".*\.h".*' "$1"/*.i | awk -F '"' '{print $2}' | sort | uniq >> $DEPENDENCY

    if [ $? -ne 0 ] ; then
        echo "Failed to extract dependencies."
        rm -f "${DEPENDENCY}"
        return 1
    fi

    # Make sure the destination file doesn't exist.
    rm -f "$2"
    dirlen=$(readlink -f "${source_dir}" | wc -c)

    # Loop through all headers.
    while read line; do
        echo "$line" | grep ^/ > /dev/null 2>&1

        # Absolute path?
        if [ $? -eq 0 ] ; then
            echo "$line" | grep ^"$(readlink -f "${source_dir}")" > /dev/null 2>&1

            # Common Headers
            if [ $? -eq 0 ]; then
                path_end=$(echo $line | tail -c +$dirlen)
                sum=$(md5sum "${line}" | cut -d ' ' -f 1 2>/dev/null)

                if [ $? -ne 0 ] ; then
                        echo "Failed to get checksum for file: ${linkfile}"
                        echo "Unable to continue."
                        rm -f "${DEPENDENCY}"
                        return 1
                fi

                echo "${sum} source${path_end}" >> "$2"
            fi

        # Generated Headers (not absolute path)
        else
            sum=$(md5sum "${kernel}/${line}" | cut -d ' ' -f 1 2>/dev/null)

            if [ $? -ne 0 ] ; then
                echo "Failed to get checksum for file: ${kernel}/${line}"
                echo "Unable to continue."
                rm -f "${DEPENDENCY}"
                return 1
            fi

            echo "${sum} generated/${line}" >> "$2"
        fi
    done < ${DEPENDENCY}

    rm "${DEPENDENCY}"
    return 0
}

#
# Build dependency module and produce header checksums
# based on the build output .i file.
#
gen_header_checksums() {
    if [ -f "${cache_dir}/pkgtmp/dependency_mod/env" ] ; then
        . "${cache_dir}/pkgtmp/dependency_mod/env"
    else
        echo "No build environment found, unable to produce header checksums."
        return 1
    fi

    if [ -z "${KERNEL_COMPILER}" ] ; then
        KERNEL_COMPILER=${CROSS_COMPILE}gcc
    fi

    mute env
    mute command -v ${KERNEL_COMPILER}

    if [ $? -ne 0 ] ; then
        echo "${KERNEL_COMPILER} not found. Do you have the compiler in PATH?"
        return 1
    fi

    if [ -z "${KERNEL_LINKER}" ] ; then
        KERNEL_LINKER=${CROSS_COMPILE}ld
    fi

    mute command -v ${KERNEL_LINKER}

    if [ $? -ne 0 ] ; then
        echo "${KERNEL_LINKER} not found. Do you have the linker in PATH?"
        return 1
    fi

    make -C "$kernel" ${CACHE_MAKE_FLAGS} M="${cache_dir}/pkgtmp/dependency_mod" depmod.i > $dbgdev
    
    if [ $? -ne 0 ] ; then
        echo "Compilation failed. Unable to compute header dependency tree."
        return 1
    fi

    # Compute checksums from headers present in the depmod.i file.
    calc_header_checksums "${cache_dir}/pkgtmp/dependency_mod" "$1"
    return $?
}

#
# Match kernel symbol CRCs.
#
check_symvers() {
    searchdir="$1"
    symvers="$2"

    searchvers=$(mktemp)
    symvers_parsed=$(mktemp)

    sort $(find "$searchdir" -name \*.mod.c) | uniq | \
        sed -rn 's/^[[:space:]]*\{[[:space:]]*0x([[:xdigit:]]{8}),.*["(](.*)[")].*\}.*/0x\1 \2/p' > $searchvers

    awk '{print $1,$2}' "$symvers" > "$symvers_parsed"

    sort "$symvers_parsed" "$searchvers" | uniq -d | diff "$searchvers" - > /dev/null
    ret=$?

    rm -f "$symvers_parsed" "$searchvers"

    return $ret
}

#
# Remove files related to given cache entry.
#
destroy_cache_entry() {
    echo "Destroying cache entry: ${1}"
    rm "${cache_dir}/${pkg}".{pkg,md5sum,target,pkgname}
}

#
# Match cache entries against given --source-dir/--output-dir
#
lookup_cache() {
    if [ ! -f "${kernel}/Module.symvers" ] ; then
        echo "${kernel}/Module.symvers does not exist. Unable to lookup cache."
        return 1
    fi

    # All cache entries. Prioritize recently used entries.
    cachefiles=$(cd "${cache_dir}"; ls -t *.pkg 2>/dev/null)

    if [ $? -ne 0 ] ; then
        echo "Can't find any cache files."
        return 1
    fi

    for pkg in ${cachefiles} ; do
        pkg=$(basename "$pkg" .pkg)
        echo -n "Cache lookup: ${pkg}.pkg ... "

        if [ ! -f "${cache_dir}/${pkg}.target" -o $(cat "${cache_dir}/${pkg}.target") != "$target" ] ; then
            echo "miss (different target)"
            continue
        fi

        if [ ! -f "${cache_dir}/${pkg}.md5sum" -o ! -f "${cache_dir}/${pkg}.pkgname" ] ; then
            echo "md5sum or pkgname file missing for ${cache_dir}/${pkg}, unable to validate"
            continue
        fi

        pkgname=$(cat "${cache_dir}/${pkg}.pkgname")

        if [ -n "$check_latest" -a "$pkgname" != "$latest_pkg" ] ; then
            echo "Old cache entry: new version available"
            [ -z "$autobuild" ] || destroy_cache_entry "old version"
            continue
        fi

        # Prepare to match kernel symbol CRCs and header checksums.

        rm -rf "${cache_dir}/pkgtmp"
        mkdir "${cache_dir}/pkgtmp"
        tar xf "${cache_dir}/${pkg}.pkg" --strip-components=1 -C "${cache_dir}/pkgtmp"

        # Match symbol CRCs

        check_symvers "${cache_dir}/pkgtmp" "${kernel}/Module.symvers"
        if [ $? -ne 0 ] ; then
            echo "miss (kernel symbol CRCs differ)"
            rm -rf "${cache_dir}/pkgtmp"
            continue
        fi

        # Build dependency module and compute header checksums.
        if [ -z "${CACHE_MAKE_FLAGS}" ] ; then
            CACHE_MAKE_FLAGS="ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE $CUST_KENV"
        fi
        tmpsums=$(mktemp)
        gen_header_checksums "$tmpsums"
        if [ $? -ne 0 ] ; then
            echo "checksum calculation failed"
            rm -rf "${cache_dir}/pkgtmp"
            rm -f "$tmpsums"
            continue
        fi

        # Match header checksums.

        diff "$tmpsums" "${cache_dir}/${pkg}.md5sum" > /dev/null
        if [ $? -ne 0 ] ; then
            rm -rf "${cache_dir}/pkgtmp"
            rm -f "$tmpsums"
            echo "miss (header checksums differ)"
            continue
        fi

        rm -f "$tmpsums"

        echo "Cache hit! Relinking modules..."

        # Relink all kernel modules against currently used kernel.

        if [ -z "$(find "${cache_dir}/pkgtmp/" -name \*.ko)" ] ; then
            require_relink="yes"
        fi

        mute env
        mute command -v ${KERNEL_COMPILER}
        mute command -v ${KERNEL_LINKER}

        for driver_obj in "${cache_dir}/pkgtmp/"*/objects ; do
            driver=$(dirname "$driver_obj")

            if [ ! -f "${driver}/objects/Makefile.autobuild" ] ; then
                echo "Old cache entry: no Makefile.autobuild found"
                destroy_cache_entry "created by an old version"
                rm -rf "${cache_dir}/pkgtmp"
                continue 2
            fi

            rm -f "${driver}/objects/Kbuild"
            mv "${driver}/objects/Makefile.autobuild" "${driver}/objects/Makefile"

            if [ "${verbose}" = "yes" ] ; then
                CACHE_MAKE_FLAGS="${CACHE_MAKE_FLAGS} V=1"
            fi

            echo Relinking $(basename "${driver}") ...
            mute make -C "$kernel" ${CACHE_MAKE_FLAGS} M="${driver}/objects" modules

            if [ $? -ne 0 ] ; then
                echo "Kernel module relinking failed - this usually shouldn't happen."
                echo "Refusing to use this cache entry!"
                rm -rf "${cache_dir}/pkgtmp"
                continue 2
            fi

            echo "Stripping debug information..."

            for mod in "${driver}"/objects/*.ko ; do
                "${CROSS_COMPILE}strip" --strip-debug "${mod}" 1>/dev/null 2>/dev/null
                cp "${mod}" "${driver}/kernel-module/"
            done

            relinked="yes"
        done

        if [ -n "$require_relink" -a -z "$relinked" ] ; then
            echo "Old cache entry: no objects for relinking found"
            destroy_cache_entry "created by an old version"
            rm -rf "${cache_dir}/pkgtmp"
            continue
        fi

        # Produce a package like the ones from the server.
        echo "Packaging ${pkgname}..."
        pkgcontents=$(tar tf "${cache_dir}/${pkg}.pkg")
        origname=$(echo "${pkgcontents}" | head -n 1 | awk -F '/' '{print $1}')
        rm -rf "${cache_dir}/${origname}"
        mv "${cache_dir}/pkgtmp" "${cache_dir}/${origname}"
        tar czf "$(pwd)/${pkgname}" -C "${cache_dir}" "${origname}/"
        rm -rf "${cache_dir}/${origname}"

        # Update cache entry timestamp.
        touch "${cache_dir}/${pkg}.pkg"
        return 0
    done

    echo "No cache hits."
    return 1
}

#
# Start remote build. Headers package must be uploaded
# first.
#
do_remote_build() {
    echo "Starting remote build against target ${target}..."

    # Start build
    if [ "$http_client" = "wget" ] ; then
        reply=$(do_retry $wget --post-data="terminal=1&filename=${remote_package}&target-config=${target}&extraargs=${extraargs}&use-cache=${using_cache}&script-version=${script_version}&cache-lookup-time=${cache_lookup_time}&suid=${suid}&start-build=1" -O - https://${server})
    else
        reply=$(do_retry $curl -d terminal=1 -d filename="$remote_package" -d target-config="$target" -d extraargs="$extraargs" -d use-cache="$using_cache" -d script-version="$script_version" -d cache-lookup-time="$cache_lookup_time" -d suid="$suid" -d start-build=1 https://${server})
    fi

    if [ $? -ne 0 ] ; then
        echo "${http_client} failed. Unable to start build."
        echo "Check connectivity and username/password."
        exit 1
    fi
    
    status=$(echo "$reply" | head -n 1)

    # If status is OK, build ID should be included as well
    if [ "$status" != "OK" ] ; then
        echo "Starting the build failed. Unable to continue."
        echo "The server reported:"
        echo "$status"
        exit 1
    fi
    
    # ID of this build is given in the reply
    build_id=$(echo "$reply" | head -n 2 | tail -n 1)
    
    echo "Build started, id ${build_id}"
    echo "Polling for completion every 10 seconds..."
    
    statusurl="https://${server}/builds/${build_id}/.status"

    for i in `seq $max_polls`
    do
        if [ "$http_client" = "wget" ] ; then
            reply=$($wget_quiet -O - "$statusurl")
        else
            reply=$($curl_quiet "$statusurl")
        fi
        
        if [ $? -ne 0 ] ; then
            if [ $i -eq $max_polls ] ; then
                echo "Maximum attempts exceeded. Build process"
                echo "died or hung. Please notify Tuxera if the"
                echo "problem persists."
                exit 1
            fi

            echo "Not finished yet; waiting..."
            sleep $poll_delay
            continue
        fi
        
        break
    done
    
    echo "Build finished."

    # Downloadable filename given in the reply
    status=$(echo "$reply" | head -n 1)
    
    if [ "$status" != "OK" ] ; then
        echo "Build failed. Cannot download package."
        echo "Tuxera has been notified of this failure."
        exit 1
    fi
    
    filename=$(echo "$reply" | head -2 | tail -1)
    fileurl="https://${server}/builds/${build_id}/${filename}"
    
    echo "Downloading ${filename} ..."
    
    if [ "$http_client" = "wget" ] ; then
        reply=$(do_retry $wget -O "$filename" "$fileurl")
    else
        reply=$(do_retry $curl -o "$filename" "$fileurl")
    fi
    
    if [ $? -ne 0 ] ; then
        echo "Failed. You can still try to download using the link in the e-mail that was sent."
        exit 1
    fi
    
    echo "Download finished."

    # Create cache entry.
    if [ -n "$use_cache" ] ; then
        echo "Updating cache..."

        pkgprefix="$(date +%Y-%m-%d-%H-%M-%S)-$(head -c 8 /dev/urandom | md5sum | head -c 4)"
        cp "$filename" "${cache_dir}/${pkgprefix}.pkg"
        echo "$target" > "${cache_dir}/${pkgprefix}.target"
        echo "$filename" > "${cache_dir}/${pkgprefix}.pkgname"

        # Prepare to compute checksums.

        mkdir "${cache_dir}/pkgtmp"
        tar xf "${cache_dir}/${pkgprefix}.pkg" --strip-components=1 -C "${cache_dir}/pkgtmp"

        # Build dependency module and compute checksums.

        gen_header_checksums "${cache_dir}/${pkgprefix}.md5sum"
        if [ $? -ne 0 ] ; then
            echo "Updating cache failed."
            rm -rf "${cache_dir}/${pkgprefix}.pkg"
            rm -f "${cache_dir}/${pkgprefix}.target"
        fi

        rm -rf "${cache_dir}/pkgtmp"

        # Purge cache entries if there are more than max_cache_entries.
        cachefiles=$(cd "${cache_dir}"; ls -t *.pkg 2>/dev/null | tail -n +$((${max_cache_entries}+1)))

        for pkg in ${cachefiles} ; do
            pkg=$(basename "$pkg" .pkg)
            destroy_cache_entry "enforcing cache size limit"
        done
    fi
}

#
# Request a list of available targets on the server.
#
list_targets() {
    echo "Connecting..."

    if [ "$http_client" = "wget" ] ; then
        reply=$(do_retry $wget --post-data="suid=${suid}" -O - https://${server}/targets.php)
    else
        reply=$(do_retry $curl -d suid="$suid" https://${server}/targets.php)
    fi

    if [ $? -ne 0 ] ; then
        echo "Unable to list targets. Check connectivity and username/password."
        exit 1
    fi

    echo
    echo "Available targets for this user:"
    echo "$reply"
    echo
}

#
# Request the latest package name from the server, for a given
# target. Used to validate cache entry freshness.
#
get_latest() {
    echo "Checking for latest release..."

    if [ "$http_client" = "wget" ] ; then
        latest_pkg=$(do_retry $wget --post-data="target-config=${target}&suid=${suid}" -O - https://${server}/latest.php)
    else
        latest_pkg=$(do_retry $curl -d target-config="$target" https://${server}/latest.php)
    fi

    if [ $? -ne 0 ] ; then
        echo "Unable to get latest release. Check connectivity and username/password."
        exit 1
    fi

    if [ "$latest_pkg" = "FAIL" -o -z "$latest_pkg" ] ; then
        echo "Unable to get latest release for this target."
        echo "Use '-a --target list' to get valid targets."
        exit 1
    fi

    echo "Latest release is ${latest_pkg}"
}

#
# Randomly pick a live Autobuild server to use.
#
select_server() {
    # for addr in `shuf -e $autobuild_addrs`; do
    local status
    local result
    for addr in $autobuild_addrs; do
        echo "Trying $addr ..." >&2

        if [ "$http_client" = "wget" ] ; then
            result=$($wget -O - --connect-timeout=15 "https://${addr}/node_select.php")
        else
            result=$($curl --connect-timeout 15 "https://${addr}/node_select.php")
        fi
        status=$?
        [ $status -eq 0 ] && break
    done
    echo "$result"
    return $status
}

#
# Find out if wget or curl is available, and ask for
# credentials if they haven't been supplied.
# Pick a live Autobuild server to use.
#
check_http_client() {
    while [ -z "$username" ] ; do
        echo -n "Please enter your username: "
        read username
    done

    while [ -z "$password" ] ; do
        oldstty=$(stty -g)
        echo -n "Please enter your password: "
        stty -echo
        read password
        stty "$oldstty"
        echo
    done

    suid=${username}
    [ -z "${admin}" ] || username="${admin}"

    curl_quiet="curl --retry ${retry} --retry-delay ${retry_delay}"
    curl_quiet=${curl_quiet}" --retry-max-time ${retry_max_time}"
    curl_quiet=${curl_quiet}" -f -u ${username}:${password}"
    wget="wget --user ${username} --password ${password}"
    wget_quiet=${wget}

    if [ -z "${verbose}" ] ; then
        curl_quiet=${curl_quiet}" -s"
        wget_quiet=${wget_quiet}" -q"
        wget=${wget}" -nv"
    fi
    
    if [ -n "$ignore_certificates" ] ; then
        curl_quiet=${curl_quiet}" -k"
        wget_quiet=${wget_quiet}" --no-check-certificate"
        wget=${wget}" --no-check-certificate"
    fi

    curl=${curl_quiet}" -S"

    if [ -n "$http_client" ] ; then
        echo "HTTP client forced to ${http_client}"
    else
        http_client="curl"
        echo -n "Checking for 'curl'... "
        command -v curl > /dev/null

        if [ $? -ne 0 ] ; then
            echo "no."
            http_client="wget"
            echo -n "Checking for 'wget'... "
            command -v wget > /dev/null
        fi

        if [ $? -ne 0 ] ; then
            echo "no. Unable to continue."
            exit 1
        fi

        echo "yes."
    fi

    if [ -n "$server" ] ; then
        echo "Server forced to $server"
    else
        server=$(do_retry select_server)
        if [ $? -ne 0 ] ; then
            echo "Unable to contact Autobuild."
            echo "Check connectivity and username/password."
            exit 1
        else
            echo "OK, using $server"
        fi
    fi
}

#
# Check if given commands (in parameters) exist
#
check_cmds() {
    echo -n "Checking for: "

    for c in $* ; do
        echo -n "$c "
        command -v $c > /dev/null

        if [ $? -ne 0 ] ; then
            echo "... no."
            echo "Unable to continue."
            exit 1
        fi
    done

    echo "... yes."
    return 0
}

#
# Check if required commands exist for Autobuild
#
check_autobuild_prerequisites() {
    check_cmds date stty mktemp chmod tail head md5sum basename

    # perl is needed with wget for urlencode
    if [ "$http_client" = "wget" ] ; then
        echo -n "Checking for 'perl'... "

        command -v perl > /dev/null

        if [ $? -ne 0 ] ; then
            echo "no. Unable to continue."
            exit 1
        fi

        echo "yes."
    fi

    if [ -n "$use_cache" ] ; then
        check_cmds egrep awk touch uniq sort tr diff make \
            dirname wc
    fi
}

#
# Upgrade to latest script
#
upgrade() {
    check_http_client
    check_cmds mktemp
    upgrade_url="https://$server/tuxera_update.sh"

    tmpscript=$(mktemp)

    if [ "$http_client" = "wget" ] ; then
        reply=$(do_retry $wget -O "$tmpscript" "$upgrade_url")
    else
        reply=$(do_retry $curl -o "$tmpscript" "$upgrade_url")
    fi

    if [ $? -ne 0 ] ; then
        echo "Upgrade failed."
        exit 1
    fi

    mv "$tmpscript" "$0"

    echo "Before upgrade: tuxera_update.sh version $script_version"
    echo "After upgrade: $(sh $0 -v)"
}

#
# Decide on a value for source_dir
#
set_source_dir() {
    if [ -n "$output_dir" ]; then
        for f in Makefile Module.symvers; do
            if ! [ -e "${output_dir}/$f" ]; then
                echo "Can't find ${output_dir}/$f - this is not a valid kernel build directory."
                exit 1
            fi
        done
        if [ -z "$source_dir" ]; then
            source_dir=$(sed -n 's/^\s*MAKEARGS\s*:=.*-C\s*\(\S\+\).*/\1/p' "${output_dir}/Makefile" 2>/dev/null)
            if [ -z "$source_dir" ]; then
               source_dir="$output_dir"
            else
               source_dir=$(cd "$output_dir" 2>/dev/null && readlink -f "$source_dir" || echo "$source_dir")
            fi
            if ! [ -e "$source_dir" ]; then
                echo "Unable to parse kernel source directory from --output-dir Makefile."
                echo "You must specify --source-dir."
                exit 1
            fi
            echo "Using ${source_dir} as kernel source directory (based on ${output_dir}/Makefile)"
            echo "Use --source-dir to override this."
        fi
    fi

    # This variable will be used like make -C $kernel
    kernel="$source_dir"

    if [ -n "$output_dir" ] ; then
        kernel="$output_dir"
    fi
}

mute() {
  if [ "${verbose}" = "yes" ] ; then
    "$@"
  else
    "$@" >/dev/null 2>&1
  fi
}

#
# Script start
#

script_version="19.12.17"
cache_dir=".tuxera_update_cache"
dbgdev="/dev/null"
cache_lookup_time="none"
max_cache_entries=10
default_polling_time_sec=300
poll_delay=10
declare -i max_polls=$default_polling_time_sec/$poll_delay
autobuild_addrs="autobuild-1.tuxera.com autobuild-2.tuxera.com"
retry=5
retry_delay=2
retry_max_time=120
conn_fail_retry=6
conn_fail_retry_delay=4

echo "tuxera_update.sh version $script_version"

if ! options=$(getopt -o pahuv -l target:,user:,pass:,use-package:,source-dir:,output-dir:,version:,cache-dir:,server:,extraargs:,max-cache-entries:,admin:,upgrade,help,ignore-cert,no-check-certificate,use-curl,use-wget,no-excludes,use-cache,verbose,latest,retry-on-connection-fail,disable-kh-check,max_build_time_sec,path-append:,path-prepend: -- "$@")
then
    usage
fi

eval set -- "$options"

while [ $# -gt 0 ]
do
    case $1 in
    -p) pkgonly="yes" ;;
    -a) autobuild="yes" ;;
    -v) exit 0 ;;
    --target) target="$2" ; shift;;
    --user) username="$2" ; shift;;
    --pass) password="$2" ; shift;;
    --use-package) local_package="$2" ; shift;;
    --source-dir) source_dir="$2" ; shift;;
    --output-dir) output_dir="$2" ; shift;;
    --ignore-cert) ignore_certificates="yes" ;;
    --no-check-certificate) ignore_certificates="yes" ;;
    --use-wget) http_client="wget" ;;
    --use-curl) http_client="curl" ;;
    --extraargs) extraargs="$2" ; shift;;
    --no-excludes) no_excludes="yes" ;;
    --use-cache) use_cache="yes" ;;
    --cache-dir) cache_dir="$2" ; shift;;
    --server) server="$2" ; shift;;
    --help | -h) long_help="yes"; usage;;
    --upgrade | -u) do_upgrade="yes" ;;
    --verbose) verbose="yes"; dbgdev="/dev/stdout" ;;
    --latest) check_latest="yes" ;;
    --max-cache-entries) max_cache_entries="$2" ; shift;;
    --retry) retry="$2" ; shift;;
    --retry-delay) retry_delay="$2" ; shift;;
    --retry-max-time) retry_max_time="$2" ; shift;;
    --admin) admin="$2" ; shift;;
    --retry-on-connection-fail) retry_on_conn_fail="yes" ;;
    --max_build_time_sec) max_build_time_sec="$2" ; shift;;
    --disable-kh-check) disable_kernel_headers_check="yes" ;;
    --path-append) path_append="$2"; shift;;
    --path-prepend) path_prepend="$2"; shift;;
    (--) shift; break;;
    (-*) echo "$0: error - unrecognized option $1" 1>&2; usage;;
    (*) break;;
    esac
    shift
done

# set max_polls in case customer provided max_build_time_sec > default_polling_time_sec
if [ ! -z "$max_build_time_sec" ] ; then
    if [ $max_build_time_sec -gt $default_polling_time_sec ] ; then
        echo "Using '$max_build_time_sec' as max_build_time_sec"
        max_polls=$max_build_time_sec/$poll_delay
    else
        echo "Using default max_build_time_sec: '$default_polling_time_sec'"
    fi
fi

# Retry enabled only in case --use-cache is set
# (assume high-load clients use --use-cache)
[ -n "$use_cache" ] && retry_on_conn_fail="yes"

# double max_polls if retry_on_conn_fail enabled
[ -n "$retry_on_conn_fail" ] && max_polls=$(($max_polls * 2))

if [ -n "$do_upgrade" ] ; then
    upgrade
    exit 0
fi

if [ -z "$target" ] ; then
    target="default"
fi

if [ -n "$path_append" ] ; then
	export PATH=${PATH}:"$path_append"
    echo Using modified PATH: ${PATH}
fi

if [ -n "$path_prepend" ] ; then
	export PATH="$path_prepend":${PATH}
    echo Using modified PATH: ${PATH}
fi

# Temporary disable path restrictions on android
export TEMPORARY_DISABLE_PATH_RESTRICTIONS=y

check_cmds tar find grep readlink cut sed
# Check specified output-dir exists
if [ -n "$output_dir" ] && [ ! -d "$(readlink -f "$output_dir")" ]; then
    echo "Specified --output_dir '$output_dir' doesn't exist. Unable to continue."
    exit 1
fi

# Check specified source-dir exists
if [ -n "$source_dir" ] && [ ! -d "$(readlink -f "$source_dir")" ]; then
    echo "Specified --source_dir '$source_dir' doesn't exist. Unable to continue."
    exit 1
fi

set_source_dir

if [ -n "$pkgonly" ] && [ -n "$autobuild" -o -n "$use_cache" ] ; then
    echo "You cannot specify -p with -a or --use-cache."
    usage
fi

if [ -n "$local_package" ] && [ -n "$use_cache" ] ; then
    echo "You cannot specify --use-package with --use-cache."
    usage
fi

# Do cache lookup now unless only listing is requested
if [ -n "$use_cache" -a "$target" != "list" ] ; then
    check_autobuild_prerequisites

    # Directory may or may not exist
    mkdir -p "${cache_dir}"

    # Absolute path to cache_dir. Needed by kernel build system.
    cache_dir=$(readlink -f "$cache_dir")

    if [ $(echo $cache_dir | wc -w) != "1" ] ; then
        echo "Linux build system does not support module paths with whitespace."
        exit 1
    fi

    if [ -z "$source_dir" ] ; then
        echo "You must specify kernel output (and source, if needed) dir to use the cache."
        usage
    fi

    # Request latest version from server
    if [ -n "$check_latest" ] ; then
        check_http_client
        get_latest
    fi

    cache_lookup_start=$(date '+%s')
    lookup_cache

    # Exit on cache hit
    if [ $? -eq 0 ] ; then
        exit 0
    else
        cache_lookup_time=$(($(date '+%s') - $cache_lookup_start))

        if [ -n "$autobuild" ] ; then
            echo "Proceeding with remote build..."
        else
            echo "No cache hit found. Exiting with status 2."
            exit 2
        fi
    fi
fi

if [ -n "$autobuild" ] ; then
    # Prerequisites may already have been checked
    [ -n "$use_cache" ] || check_autobuild_prerequisites
    [ -n "$use_cache"  -a -n "$check_latest" ] || check_http_client

    if [ "$target" = "list" ] ; then
        list_targets
        exit 0
    fi

    # Assemble package with generated name if no local_package set
    if [ -z "$local_package" ] ; then
        local_package="kheaders_$(date +%Y-%m-%d-%H-%M-%S-$(head -c 8 /dev/urandom | md5sum | head -c 4)).tar.bz2"
        build_package "$local_package"
    fi

    if [ "$http_client" = "wget" ] ; then
        upload_package_wget "$local_package"
    else
        upload_package_curl "$local_package"
    fi

    # Note: use_cache must remain empty in case of 'no'
    using_cache=$use_cache
    if [ -z "$use_cache" ] ; then
        using_cache="no"
    fi

    do_remote_build "$remote_package"
    exit 0
fi

build_package "kheaders.tar.bz2"
exit 0
