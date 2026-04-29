#include "vp9_parser.h"
#include "vp9_dpb.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
#include <vector>
static std::vector<uint8_t> read_file(const char *p) {
    FILE *f = fopen(p,"rb"); if(!f) return{};
    fseek(f,0,SEEK_END); long n=ftell(f); fseek(f,0,SEEK_SET);
    std::vector<uint8_t> v(n); fread(v.data(),1,n,f); fclose(f);
    return v;
}
int main(int argc, char **argv) {
    auto ivf = read_file(argv[1]);
    vp9::ParserState st{}; vp9::PicParams pp{}; vp9::ProbUpdates pu{};
    uint32_t fsz; memcpy(&fsz, ivf.data()+32, 4);
    const uint8_t *frame = ivf.data()+32+12;
    const uint8_t *frames[8]={}; size_t sizes[8]={};
    vp9::Vp9Parser_SuperframeSplit(frame, fsz, frames, sizes, 8);
    vp9::Vp9Parser_Parse(frames[0], sizes[0], st, pp, pu);
    printf("frame_type=%d intra_only=%d tx_mode=%d lossless=%d\n", pp.frame_type, pp.intra_only, pu.tx_mode, pp.lossless);
    printf("tx_probs_present=%d skip_present=%d coef_present[0..3]=%d %d %d %d\n",
        pu.tx_probs_present, pu.skip_present,
        pu.coef_present[0], pu.coef_present[1], pu.coef_present[2], pu.coef_present[3]);
    printf("skip_flag=%d,%d,%d  skip=%d,%d,%d\n",
        pu.skip_flag[0],pu.skip_flag[1],pu.skip_flag[2],
        pu.skip[0],pu.skip[1],pu.skip[2]);
    printf("tx8p_flag=%d,%d  tx16p_flag={%d,%d},{%d,%d}  tx32p_flag={%d,%d,%d},{%d,%d,%d}\n",
        pu.tx_size_8x8_flag[0][0],pu.tx_size_8x8_flag[1][0],
        pu.tx_size_16x16_flag[0][0],pu.tx_size_16x16_flag[0][1],
        pu.tx_size_16x16_flag[1][0],pu.tx_size_16x16_flag[1][1],
        pu.tx_size_32x32_flag[0][0],pu.tx_size_32x32_flag[0][1],pu.tx_size_32x32_flag[0][2],
        pu.tx_size_32x32_flag[1][0],pu.tx_size_32x32_flag[1][1],pu.tx_size_32x32_flag[1][2]);
    int cnt=0; for (int a=0;a<4;a++)for(int b=0;b<2;b++)for(int c=0;c<2;c++)
        for(int d=0;d<6;d++)for(int e=0;e<6;e++)for(int f=0;f<3;f++)
            if (pu.coef_changed[a][b][c][d][e][f]) cnt++;
    printf("coef_changed total set: %d\n", cnt);
    return 0;
}
