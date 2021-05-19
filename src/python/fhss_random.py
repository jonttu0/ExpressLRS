import random
import os

DEBUG = 0

CALC_MY_STEP = 1

FHSS_FREQS_HEAD = '''
#pragma once

#include "platform.h"
#include <stdint.h>

'''

FHSS_FREQS_TAIL = '''
'''

RNG_MAX = 0x7FFF
seed = 0
use_local_rand = 1
rand_version = 1

# Fill the FHSSsequence with channel indices
# The 0 index is special - the 'sync' channel. The sync channel appears every
# syncInterval hops. The other channels are randomly distributed between the
# sync channels
SYNC_INTERVAL = 37 #25

# Size of the FHSS sequence entries
#   TODO: modify bigger at some day (e.g 20 * SYNC_INTERVAL).
#         ElrsSyncPacket_s need to be updated to hold bigger than uin8_t value!!
NR_SEQUENCE_ENTRIES = 256

# returns values between 0 and 0x7FFF
# NB rngN depends on this output range, so if we change the
# behaviour rngN will need updating
def rng():
    global seed
    m = 2147483648
    a = 214013
    c = 2531011
    seed = (a * seed + c) % m
    return seed >> 16

def rngSeed(newSeed):
    global seed
    seed = newSeed
    random.seed(newSeed)

# returns 0 <= x < max where max <= 256
# (actual upper limit is higher, but there is one and I haven't
#  thought carefully about what it is)
def rngN(max):
    if use_local_rand:
        x = rng()
        result = (x * max) / RNG_MAX
        return int(result)
    return random.randrange(0, max, 1)


def CalcCRC32(data, length=None):
    if length is None:
        length = len(data)
    crc = 0xffffffff
    for i in range(length):
        crc = crc ^ data[i]
        for j in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1))
    return 0xffffffff & ~crc


def print_fhss(vals, num_of_fhss):
    buffer = []
    print_lst = []
    i = 0
    for val in vals:
        print_lst.append("%2u" % val)

        if ((i + 1) % 16 == 0):
            buffer.append(", ".join(print_lst))
            print(" ".join(print_lst))
            print_lst = []
        i += 1
    if print_lst:
        buffer.append(", ".join(print_lst))
        print(" ".join(print_lst))

    for iter in range(num_of_fhss):
        print("  index %2d: %3u" % (iter, vals.count(iter)))
    return buffer


def FHSSrandomiseFHSSsequence_v1(num_of_fhss, sync_interval):

    def resetIsAvailable():
        l = [1] * num_of_fhss
        if sync_interval is not None:
            l[0] = 0
        return l

    isAvailable = resetIsAvailable()
    FHSSsequence = [0] * NR_SEQUENCE_ENTRIES
    prev = 0 # needed to prevent repeats of the same index

    # for each slot in the sequence table
    for i in range(NR_SEQUENCE_ENTRIES):
        if sync_interval is not None and ((i % sync_interval) == 0):
            # assign sync channel 0
            FHSSsequence[i] = 0
            prev = 0
        else:
            # pick one of the available channels. May need to loop to avoid repeats
            index = prev
            while (index == prev): # can't use index if it repeats the previous value
                c = rngN(isAvailable.count(1)) # returnc 0<c<nLeft
                # find the c'th entry in the isAvailable array
                # can skip 0 as that's the sync channel and is never available for normal allocation
                index = 1 if sync_interval is not None else 0
                found = 0
                while (index < num_of_fhss):
                    if (isAvailable[index]):
                        if (found == c):
                            break
                        found += 1
                    index += 1

                if (index == num_of_fhss):
                    # This should never happen
                    print("FAILED to find the available entry for '%s'!" % c)
                    # What to do? We don't want to hang as that will stop us getting to the wifi hotspot
                    # Use the sync channel
                    index = 0
                    break

            FHSSsequence[i] = index  # assign the value to the sequence array
            isAvailable[index] = 0   # clear the flag
            prev = index             # remember for next iteration
            if (isAvailable.count(1) == 0):
                # we've assigned all of the channels, so reset for next cycle
                isAvailable = resetIsAvailable()
    return FHSSsequence


def FHSSrandomiseFHSSsequence_v2(num_of_fhss, sync_interval):
    vals = [-1] * NR_SEQUENCE_ENTRIES
    index = -1
    for iter in range(NR_SEQUENCE_ENTRIES):
        # dont use same value if it is close to last three
        skip = [vals[iter - 3], vals[iter - 2], vals[iter - 1]]
        while (index in skip or
                ((index - 1) % NR_SEQUENCE_ENTRIES) in skip or
                ((index + 1) % NR_SEQUENCE_ENTRIES) in skip):
            index = rngN(num_of_fhss)
        vals[iter] = index
    return vals


def check_fhss_freqs_h(DOMAIN, MY_UID):
    global SYNC_INTERVAL

    _uid = [int(val, 16) for val in MY_UID.replace("-DMY_UID=", "").split(",")]
    _uid_crc = CalcCRC32(bytearray(_uid))

    DOMAIN = DOMAIN.replace("-D", "")
    ISM_band = 0
    if "_ISM_2400" in DOMAIN:
        ISM_band = 1
        if "_FLRC" in DOMAIN:
            ISM_band = 2


    outfile_name = ['fhss_freqs_127x.h', 'fhss_freqs_128x.h', 'fhss_freqs_128x_flrc.h'][ISM_band]

    FREQ_OFFSET_UID = sum(_uid[3:])

    # Our table of FHSS frequencies. Define a regulatory domain to select the correct set for your location and radio
    if DOMAIN == "Regulatory_Domain_AU_433":
        FHSSfreqs = [
            433420000,
            433920000,
            434420000
        ]

    elif DOMAIN == "Regulatory_Domain_EU_433":
        '''
         Frequency band G, taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
         Note: As is the case with the 868Mhz band, these frequencies only comply to the license free portion
               of the spectrum, nothing else. As such, these are likely illegal to use.
        '''
        FHSSfreqs = [
            433100000,
            433925000,
            434450000
        ]

    elif DOMAIN == "Regulatory_Domain_AU_915":
        FHSSfreqs = [f for f in range(915500000, 927000000, 600000)]

    elif DOMAIN == "Regulatory_Domain_FCC_915":
        # Very definitely not fully checked. An initial pass at increasing the hops
        FHSSfreqs = [f for f in range(903500000, 927000000, 600000)]

    elif DOMAIN == "Regulatory_Domain_EU_868":
        '''
        s/* Frequency bands taken from https://wetten.overheid.nl/BWBR0036378/2016-12-28#Bijlagen
        * Note: these frequencies fall in the license free H-band, but in combination with 500kHz
        * LoRa modem bandwidth used by ExpressLRS (EU allows up to 125kHz modulation BW only) they
        * will never pass RED certification and they are ILLEGAL to use.
        *
        * Therefore we simply maximize the usage of available spectrum so laboratory testing of the software won't disturb existing
        * 868MHz ISM band traffic too much.
        */
        '''
        # 863275000 = band H1, 863 - 865MHz, 0.1% duty cycle or CSMA techniques, 25mW EIRP
        # 865375000 = Band H2, 865 - 868.6MHz, 1.0% dutycycle or CSMA, 25mW EIRP
        # 868525000 = Band H3, 868.7-869.2MHz, 0.1% dutycycle or CSMA, 25mW EIRP
        FHSSfreqs = [f for f in range(863275000, 872500000, 525000)]

    elif DOMAIN == "Regulatory_Domain_EU_868_R9":
        # https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/4ae30dc3b049f18147d6e278817f7a5f425c2fb0/Multiprotocol/FrSkyR9_sx1276.ino#L43
        #FHSSfreqs = [f for f in range(859504640, 872500000, 499712)]
        FHSSfreqs = [f for f in range(859504640, 872500000, 500000)]

    elif DOMAIN == "Regulatory_Domain_ISM_2400":
        # These are for 1625kHz band
        FHSSfreqs = [f for f in range(2400400000, 2480000000, 1650000)]
        SYNC_INTERVAL = 65 #49

    elif DOMAIN == "Regulatory_Domain_ISM_2400_800kHz":
        # These are for 812.5kHz band
        #FHSSfreqs = [f for f in range(2400400000, 2480000000, 1000000)]
        #FHSSfreqs = [f for f in range(2400400000, 2480000000, 900000)]
        FHSSfreqs = [f for f in range(2400400000, 2480000000, 850000)]
        SYNC_INTERVAL = 65

    elif DOMAIN == "Regulatory_Domain_ISM_2400_FLRC":
        # These are for 300kHz band
        FHSSfreqs = [f for f in range(2400400000, 2480000000, 330000)]
        SYNC_INTERVAL = 65

    else:
        raise Exception("[error] No regulatory domain defined, please define one in common.h")
        return

    num_of_fhss = len(FHSSfreqs)
    print("Number of FHSS frequencies = %u" % num_of_fhss)
    freq_base = FHSSfreqs[0]
    bandwidth = FHSSfreqs[1] - FHSSfreqs[0]
    print("Freq base: %u, BW: %u" % (freq_base, bandwidth))

    rngSeed(_uid_crc)

    header = "// MY_UID=%s; DOMAIN=%s; RAND=%u.%u; SYNC_INTERVAL=%s; CALC_MY_STEP=%u\n" % (
        MY_UID, DOMAIN, rand_version, use_local_rand, SYNC_INTERVAL, CALC_MY_STEP)
    write_out = False
    try:
        with open(os.path.join('src', outfile_name), "r") as _f:
            first = _f.readline()
            write_out = first != header
            _f.close()
    except IOError:
        write_out = True

    if write_out or DEBUG:
        print("write...")
        with open(os.path.join('src', outfile_name), "w+") as _f:
            _f.write(header)
            _f.write(FHSS_FREQS_HEAD)

            namespace = ["SX127x", "SX128x", "SX128x_FLRC"][ISM_band]
            _f.write("namespace %s {\n\n" % namespace)

            _f.write("constexpr uint32_t FREQ_OFFSET_UID = %u;\n" % FREQ_OFFSET_UID)
            _f.write("constexpr uint32_t FREQ_BASE = %u;\n" % (freq_base + FREQ_OFFSET_UID))
            _f.write("constexpr uint32_t FREQ_BANDWIDTH = %u;\n" % bandwidth)
            _f.write("constexpr uint32_t FREQ_BAND_COUNT = %u;\n" % num_of_fhss)
            _f.write("constexpr uint32_t NR_SEQUENCE_ENTRIES = %u;\n" % NR_SEQUENCE_ENTRIES)
            _f.write("constexpr uint32_t UID_CRC32 = 0x%08X;\n" % _uid_crc)

            my_step = 0
            if CALC_MY_STEP:
                my_step = _uid_crc % 8
            my_step |= 1
            _f.write("constexpr uint32_t FHSS_MY_STEP = %u;\n\n" % (my_step))

            _f.write('static uint8_t DRAM_FORCE_ATTR FHSSsequence[NR_SEQUENCE_ENTRIES] = {\n')
            if rand_version == 1:
                FHSSsequence = FHSSrandomiseFHSSsequence_v1(num_of_fhss, SYNC_INTERVAL)
            elif rand_version == 2:
                FHSSsequence = FHSSrandomiseFHSSsequence_v2(num_of_fhss, SYNC_INTERVAL)
            else:
                raise Exception("Unknown FHSS rand version!")
            hops = print_fhss(FHSSsequence, num_of_fhss)
            for line in hops:
                _f.write("    %s,\n" % line)
            _f.write("};\n\n")

            _f.write("/* Note: UID offset included */\n")
            _f.write("/*\n")
            _f.write('static uint32_t DRAM_FORCE_ATTR FHSSfreqs[%u] = {\n' % num_of_fhss)
            freqs = ["    %u" % (freq + FREQ_OFFSET_UID) for freq in FHSSfreqs]
            _f.write(",\n".join(freqs))
            _f.write("\n};\n")
            _f.write("//*/\n\n")

            _f.write('/*\n Usage per freq:\n')
            for iter in range(num_of_fhss):
                _f.write("    index %2d: %3u\n" % (iter, FHSSsequence.count(iter)))
            _f.write('*/\n')
            _f.write("\n} /* namespace */\n")
            _f.write(FHSS_FREQS_TAIL)
            _f.close()


def check_env_and_parse(build_flags):
    my_domains = []
    my_uid = ""
    for flag in build_flags:
        if "Regulatory_Domain" in flag:
            my_domains.append(flag)
        elif "MY_UID" in flag:
            my_uid = flag

    if my_domains and my_uid:
        for domain in my_domains:
            check_fhss_freqs_h(domain, my_uid)
    else:
        raise Exception("Domain or UID is missing!")

if DEBUG:
    check_fhss_freqs_h("-DRegulatory_Domain_EU_868", "-DMY_UID=0xB8,0x27,0xEB,0x61,0x1D,0x38")
