/* SPDX-License-Identifier: GPL-2.0-or-later */
static int
opHADDPS_a16(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    tmp.f[0] = src.f[0] + src.f[1];
    tmp.f[1] = src.f[2] + src.f[3];
    tmp.f[2] = cpu_state_high.XMM[cpu_reg].f[0] + cpu_state_high.XMM[cpu_reg].f[1];
    tmp.f[3] = cpu_state_high.XMM[cpu_reg].f[2] + cpu_state_high.XMM[cpu_reg].f[3];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHADDPS_a32(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    tmp.f[0] = src.f[0] + src.f[1];
    tmp.f[1] = src.f[2] + src.f[3];
    tmp.f[2] = cpu_state_high.XMM[cpu_reg].f[0] + cpu_state_high.XMM[cpu_reg].f[1];
    tmp.f[3] = cpu_state_high.XMM[cpu_reg].f[2] + cpu_state_high.XMM[cpu_reg].f[3];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHADDPD_a16(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    tmp.d[0] = src.d[0] + src.d[1];
    tmp.d[1] = cpu_state_high.XMM[cpu_reg].d[0] + cpu_state_high.XMM[cpu_reg].d[1];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHADDPD_a32(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    tmp.d[0] = src.d[0] + src.d[1];
    tmp.d[1] = cpu_state_high.XMM[cpu_reg].d[0] + cpu_state_high.XMM[cpu_reg].d[1];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHSUBPS_a16(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    tmp.f[0] = src.f[0] - src.f[1];
    tmp.f[1] = src.f[2] - src.f[3];
    tmp.f[2] = cpu_state_high.XMM[cpu_reg].f[0] - cpu_state_high.XMM[cpu_reg].f[1];
    tmp.f[3] = cpu_state_high.XMM[cpu_reg].f[2] - cpu_state_high.XMM[cpu_reg].f[3];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHSUBPS_a32(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    tmp.f[0] = src.f[0] - src.f[1];
    tmp.f[1] = src.f[2] - src.f[3];
    tmp.f[2] = cpu_state_high.XMM[cpu_reg].f[0] - cpu_state_high.XMM[cpu_reg].f[1];
    tmp.f[3] = cpu_state_high.XMM[cpu_reg].f[2] - cpu_state_high.XMM[cpu_reg].f[3];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHSUBPD_a16(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    tmp.d[0] = src.d[0] - src.d[1];
    tmp.d[1] = cpu_state_high.XMM[cpu_reg].d[0] - cpu_state_high.XMM[cpu_reg].d[1];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opHSUBPD_a32(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    tmp.d[0] = src.d[0] - src.d[1];
    tmp.d[1] = cpu_state_high.XMM[cpu_reg].d[0] - cpu_state_high.XMM[cpu_reg].d[1];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opADDSUBPS_a16(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    tmp.f[0] = cpu_state_high.XMM[cpu_reg].f[0] - src.f[0];
    tmp.f[1] = cpu_state_high.XMM[cpu_reg].f[1] + src.f[1];
    tmp.f[2] = cpu_state_high.XMM[cpu_reg].f[2] - src.f[2];
    tmp.f[3] = cpu_state_high.XMM[cpu_reg].f[3] + src.f[3];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opADDSUBPS_a32(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    tmp.f[0] = cpu_state_high.XMM[cpu_reg].f[0] - src.f[0];
    tmp.f[1] = cpu_state_high.XMM[cpu_reg].f[1] + src.f[1];
    tmp.f[2] = cpu_state_high.XMM[cpu_reg].f[2] - src.f[2];
    tmp.f[3] = cpu_state_high.XMM[cpu_reg].f[3] + src.f[3];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opADDSUBPD_a16(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_16(fetchdat);
    SSE_GETSRC();

    tmp.d[0] = cpu_state_high.XMM[cpu_reg].d[0] - src.d[0];
    tmp.d[1] = cpu_state_high.XMM[cpu_reg].d[1] + src.d[1];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}

static int
opADDSUBPD_a32(uint32_t fetchdat)
{
    SSE_REG src, tmp;

    fetch_ea_32(fetchdat);
    SSE_GETSRC();

    tmp.d[0] = cpu_state_high.XMM[cpu_reg].d[0] - src.d[0];
    tmp.d[1] = cpu_state_high.XMM[cpu_reg].d[1] + src.d[1];
    cpu_state_high.XMM[cpu_reg].q[0] = tmp.q[0];
    cpu_state_high.XMM[cpu_reg].q[1] = tmp.q[1];
    return 0;
}