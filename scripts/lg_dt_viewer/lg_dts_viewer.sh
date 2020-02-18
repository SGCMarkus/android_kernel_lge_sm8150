#!/bin/sh
# (c) 2013, Guilbert Lee <guilbert.lee@lge.com>
# (c) 2017, Hyeon H. Park <hyunhui.park@lge.com>
# Licensed under the terms of the GNU GPL License version 2

DTS_PATH=$1
WORK_PATH=${0%/*}
FILTER="scripts"
KERNEL_PATH=${PWD%$FILTER*}

usage() {
    echo "usage : ./lg_dts_viewer.sh [options] [dts file] | [dts path]"

    echo "    - dts path : path of dts file"
    echo "    - dts file : dts file (with full path)"
    echo
    echo "options :"
    echo "    -D  : show list of deleted node and properties"
    echo "    -K  : show list of overrided properties"
    echo
    echo "example (at Kernel root) :"
    echo "./scripts/lg_dt_viewer/lg_dts_viewer.sh arch/arm64/boot/dts/lge/msm8998-taimen-rev-0.dts"
    echo "or"
    echo "./scripts/lg_dt_viewer/lg_dts_viewer.sh arch/arm64/boot/dts/lge/"
    exit
}

while getopts dDK@h flag; do
    case $flag in
    d)
        # for debugging(hidden)
        DEBUG=debug
        ;;
    D)
	SHOW_DELETED_LIST="-D"
        ;;
    K)
	SHOW_OVERRIDED_LIST="-K"
        ;;
    @)
	SHOW_SYMBOL_LIST="-@"
        ;;
    h)
        usage
        ;;
    \?)
        usage
        ;;
    esac
done

shift $((OPTIND-1))
DTS_PATH=$1

if [ "$DTS_PATH" = "" ]; then
    usage
fi

DTS_NAME=${DTS_PATH##*/}

if [ "$DTS_NAME" = "" ]; then
    if [ ! -d "$DTS_PATH" ]; then
        echo "ERROR: directory not found"
        exit
    fi
    echo "NOTE: processing path including dts, searching dts files..."
    OUT_PATH=$(basename $DTS_PATH)
    OUT_PATH=out_$OUT_PATH
    DTS_PATH=$(find $DTS_PATH -name *.dts)
    if [ "$DTS_PATH" = "" ]; then
        echo "ERROR: There is no dts file in this path"
        exit
    fi
else
    echo "NOTE: processing indivisual dts file..."
    case "$DTS_NAME" in
    *.dts)
        ;;
    *)
        echo "NOTE: Input file is not dts file"
        exit
        ;;
    esac
    OUT_PATH=${DTS_NAME/%.dts/}
    OUT_PATH=out_$OUT_PATH
fi

if [ ! -d "$OUT_PATH" ] ; then
    mkdir $OUT_PATH
fi

for dtspath in $DTS_PATH; do
    echo "DTC $dtspath"
    dtsfile=${dtspath##*/}
    # For support #include, C-pre processing first to dts.
    gcc -E -nostdinc -undef -D__DTS__ -x assembler-with-cpp \
		-I $KERNEL_PATH/include/ \
		-I $KERNEL_PATH/arch/arm64/boot/dts/ \
		-I $KERNEL_PATH/arch/arm64/boot/dts/include/ \
		$dtspath -o $OUT_PATH/$dtsfile.preprocessing

    # Now, transfer to dts from dts with LG specific
    ${WORK_PATH}/lg_dtc -q -o $OUT_PATH/$dtsfile \
        $SHOW_SYMBOL_LIST $SHOW_DELETED_LIST $SHOW_OVERRIDED_LIST \
	-I dts -O dts -H specific -s ./$OUT_PATH/$dtsfile.preprocessing

    if [ "$DEBUG" = "debug" ]
    then
        ${WORK_PATH}/lg_dtc -q -o $OUT_PATH/$dtsfile.2 \
             $SHOW_SYMBOL_LIST $SHOW_DELETED_LIST $SHOW_OVERRIDED_LIST \
             -I dts -O dts -H specific2 -s ./$OUT_PATH/$dtsfile.preprocessing
    else
        rm ./$OUT_PATH/$dtsfile.preprocessing
    fi
done
echo "out is ./$OUT_PATH"
echo "done."
