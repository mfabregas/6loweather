#!/bin/bash

usage() { echo "6LoWeather: $0 [-d <router|6loweather>] [-p <PORT>] [-s <1|0>]" 1>&2; exit 1; }

while getopts ":d:p:s:" o; do
    case "${o}" in
        d)
            d=${OPTARG}
            ;;
        p)
            p=${OPTARG}
            ;;
        s)
            s=${OPTARG}
            ;;
        *)
            usage
            ;;
    esac
done
shift $((OPTIND-1))

if [ -z "${d}" ] || [ -z "${p}" ] || [ -z "${s}" ]; then
    usage
fi

if [ "${d}" != "router" ] && [ "${d}" != "6loweather" ]; then
    usage
fi

if [ "${d}" == "router" ]; then
  myfile="6lbr-slip-radio-contikimac-2_4ghz-ch26.bin"
else
  if [ "${s}" -eq "1" ]; then
    myfile="6loweather-llsec-disabled-mosquitto-contikimac-2_4ghz-ch26.bin"
  elif [ "${s}" -eq "0" ]; then
    myfile="6loweather-llsec-enabled-mosquitto-contikimac-2_4ghz-ch26.bin"
  else
    usage
  fi
fi

echo "Flashing ${p} port with ${myfile} binary"

python cc2538-bsl.py -e -w -v -p ${p} -a 0x202000 "${myfile}"
