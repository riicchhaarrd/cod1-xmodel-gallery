// pk3.h - minimal pk3/zip reader using raw zlib
#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <cstdio>
#include <zlib.h>

// Reads a single file from a pk3 by filename (case-insensitive).
// Returns file bytes, or empty on failure.
inline std::vector<uint8_t> pk3_read(const char *pk3path, const char *filename)
{
    FILE *f = fopen(pk3path, "rb");
    if (!f) return {};

    // Find end-of-central-directory record (EOCD)
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    if (fsize < 22) { fclose(f); return {}; }

    // Search backwards for EOCD signature 0x06054b50
    long eocd_pos = -1;
    long search_start = fsize - 22;
    long search_end   = (fsize > 65557) ? fsize - 65557 : 0;
    uint8_t buf4[4];
    for (long pos = search_start; pos >= search_end; pos--) {
        fseek(f, pos, SEEK_SET);
        fread(buf4, 1, 4, f);
        if (buf4[0]==0x50 && buf4[1]==0x4b && buf4[2]==0x05 && buf4[3]==0x06) {
            eocd_pos = pos; break;
        }
    }
    if (eocd_pos < 0) { fclose(f); return {}; }

    // Parse EOCD
    fseek(f, eocd_pos + 12, SEEK_SET);
    uint8_t eocd[8];
    fread(eocd, 1, 8, f);
    uint32_t cd_size   = eocd[0] | (eocd[1]<<8) | (eocd[2]<<16) | (eocd[3]<<24);
    uint32_t cd_offset = eocd[4] | (eocd[5]<<8) | (eocd[6]<<16) | (eocd[7]<<24);
    (void)cd_size;

    // Walk central directory entries
    fseek(f, cd_offset, SEEK_SET);
    size_t fnlen_target = strlen(filename);

    while (true) {
        uint8_t sig[4];
        if (fread(sig, 1, 4, f) < 4) break;
        if (sig[0]!=0x50||sig[1]!=0x4b||sig[2]!=0x01||sig[3]!=0x02) break;

        uint8_t cde[42];
        if (fread(cde, 1, 42, f) < 42) break;

        uint16_t method    = cde[6]  | (cde[7]  <<8);
        uint32_t comp_sz   = cde[16] | (cde[17]<<8) | (cde[18]<<16) | (cde[19]<<24);
        uint32_t uncomp_sz = cde[20] | (cde[21]<<8) | (cde[22]<<16) | (cde[23]<<24);
        uint16_t fnlen     = cde[24] | (cde[25]<<8);
        uint16_t extralen  = cde[26] | (cde[27]<<8);
        uint16_t commlen   = cde[28] | (cde[29]<<8);
        uint32_t local_off = cde[38] | (cde[39]<<8) | (cde[40]<<16) | (cde[41]<<24);

        std::vector<char> fn(fnlen + 1);
        fread(fn.data(), 1, fnlen, f);
        fn[fnlen] = '\0';
        fseek(f, extralen + commlen, SEEK_CUR);

        // Case-insensitive compare
        bool match = (fnlen == (uint16_t)fnlen_target);
        if (match) {
            for (size_t i = 0; i < fnlen_target; i++) {
                char a = fn[i], b = filename[i];
                if (a >= 'A' && a <= 'Z') a += 32;
                if (b >= 'A' && b <= 'Z') b += 32;
                if (a != b) { match = false; break; }
            }
        }

        if (match) {
            // Jump to local file header
            long saved = ftell(f);
            fseek(f, local_off + 26, SEEK_SET);
            uint8_t lh[4];
            fread(lh, 1, 4, f);
            uint16_t lfnlen   = lh[0] | (lh[1]<<8);
            uint16_t lextralen = lh[2] | (lh[3]<<8);
            fseek(f, lfnlen + lextralen, SEEK_CUR);

            // Read compressed data
            std::vector<uint8_t> comp(comp_sz);
            fread(comp.data(), 1, comp_sz, f);
            fclose(f);

            if (method == 0) {
                // Stored (no compression)
                return comp;
            } else if (method == 8) {
                // Deflate
                std::vector<uint8_t> out(uncomp_sz);
                z_stream zs{};
                zs.next_in   = comp.data();
                zs.avail_in  = comp_sz;
                zs.next_out  = out.data();
                zs.avail_out = uncomp_sz;
                if (inflateInit2(&zs, -MAX_WBITS) != Z_OK) return {};
                inflate(&zs, Z_FINISH);
                inflateEnd(&zs);
                out.resize(zs.total_out);
                return out;
            }
            return {};
        }
    }
    fclose(f);
    return {};
}

// List all entries in a pk3 that start with prefix (e.g. "xmodel/")
inline std::vector<std::string> pk3_list(const char *pk3path, const char *prefix)
{
    std::vector<std::string> result;
    FILE *f = fopen(pk3path, "rb");
    if (!f) return result;

    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    if (fsize < 22) { fclose(f); return result; }

    long eocd_pos = -1;
    long search_start = fsize - 22;
    long search_end   = (fsize > 65557) ? fsize - 65557 : 0;
    uint8_t buf4[4];
    for (long pos = search_start; pos >= search_end; pos--) {
        fseek(f, pos, SEEK_SET);
        fread(buf4, 1, 4, f);
        if (buf4[0]==0x50&&buf4[1]==0x4b&&buf4[2]==0x05&&buf4[3]==0x06) {
            eocd_pos = pos; break;
        }
    }
    if (eocd_pos < 0) { fclose(f); return result; }

    fseek(f, eocd_pos + 16, SEEK_SET);
    uint8_t tmp[4];
    fread(tmp, 1, 4, f);
    uint32_t cd_offset = tmp[0]|(tmp[1]<<8)|(tmp[2]<<16)|(tmp[3]<<24);

    size_t pfxlen = strlen(prefix);
    fseek(f, cd_offset, SEEK_SET);
    while (true) {
        uint8_t sig[4];
        if (fread(sig,1,4,f)<4) break;
        if (sig[0]!=0x50||sig[1]!=0x4b||sig[2]!=0x01||sig[3]!=0x02) break;
        uint8_t cde[42];
        if (fread(cde,1,42,f)<42) break;
        uint16_t fnlen   = cde[24]|(cde[25]<<8);
        uint16_t exlen   = cde[26]|(cde[27]<<8);
        uint16_t cmlen   = cde[28]|(cde[29]<<8);
        std::vector<char> fn(fnlen+1);
        fread(fn.data(),1,fnlen,f);
        fn[fnlen]='\0';
        fseek(f, exlen+cmlen, SEEK_CUR);
        if (fnlen >= (uint16_t)pfxlen && strncasecmp(fn.data(), prefix, pfxlen) == 0)
            result.emplace_back(fn.data());
    }
    fclose(f);
    return result;
}
