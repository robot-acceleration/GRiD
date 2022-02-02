#!/usr/bin/python3
from URDFParser import URDFParser
from RBDReference import RBDReference
from GRiDCodeGenerator import GRiDCodeGenerator
from util import parseInputs, printUsage, validateRobot, initializeValues, printErr
import copy

def main():
    URDF_PATH, DEBUG_MODE, _ = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH)

    validateRobot(robot)

    codegen = GRiDCodeGenerator(robot)
    reference = RBDReference(robot)
    q, qd, u, n = initializeValues(robot)

    print("rnea fpass err")
    (v, a, f) = codegen.test_rnea_fpass(q,qd)
    (v2, a2, f2) = reference.rnea_fpass(q,qd)
    print("v")
    printErr(v,v2,DEBUG_MODE)
    print("a")
    printErr(a,a2,DEBUG_MODE)
    print("f")
    printErr(f,f2,DEBUG_MODE)

    print("rnea bpass err")
    (c, fbp) = codegen.test_rnea_bpass(q,qd,f)
    (c2, fbp2) = reference.rnea_bpass(q,qd,f2)
    print("fbp")
    printErr(fbp,fbp2,DEBUG_MODE)
    print("c")
    printErr(c,c2,DEBUG_MODE)

    print("full rbd error")
    (c, v, a, f) = codegen.test_rnea(q,qd)
    (c2, v2, a2, f2) = reference.rnea(q,qd)
    print("v")
    printErr(v,v2,DEBUG_MODE)
    print("a")
    printErr(a,a2,DEBUG_MODE)
    print("f")
    printErr(f,f2,DEBUG_MODE)
    print("c")
    printErr(c,c2,DEBUG_MODE)

    print("minv bpass err")
    Minv, F, U, Dinv = codegen.test_minv_bpass(q)
    Minv2, F2, U2, Dinv2 = reference.minv_bpass(q)
    print("U")
    printErr(U,U2,DEBUG_MODE)
    print("Dinv")
    printErr(Dinv,Dinv2,DEBUG_MODE)
    print("F")
    printErr(F,F2,DEBUG_MODE)
    print("Minv")
    printErr(Minv,Minv2,DEBUG_MODE)

    print("minv fpass err")
    Minv = codegen.test_minv_fpass(q, Minv, F, U, Dinv)
    Minv2 = reference.minv_fpass(q, Minv2, F2, U2, Dinv2)
    print("Minv")
    printErr(Minv,Minv2,DEBUG_MODE)

    print("full minv err")    
    Minv = codegen.test_minv(q)
    Minv2 = reference.minv(q)
    print("Minv")
    printErr(Minv,Minv2,DEBUG_MODE)

    print("dRnea err")
    (dc_dq, dc_dqd, dv_dq, dv_dqd, da_dq, da_dqd, df_fp_dq, df_fp_dqd, df_dq, df_dqd) = codegen.test_rnea_grad_inner(q, qd, v, a, f)
    (dv_dq2, da_dq2, df_fp_dq2) = reference.rnea_grad_fpass_dq(q, qd, v2, a2)
    (dv_dqd2, da_dqd2, df_fp_dqd2) = reference.rnea_grad_fpass_dqd(q, qd, v2)
    df_dq2 = copy.deepcopy(df_fp_dq2)
    df_dqd2 = copy.deepcopy(df_fp_dqd2)
    dc_dq2 = reference.rnea_grad_bpass_dq(q, f2, df_dq2)
    dc_dqd2 = reference.rnea_grad_bpass_dqd(q, df_dqd2)
    print("dv/dq")
    printErr(dv_dq,dv_dq2,DEBUG_MODE)
    print("dv/dqd")
    printErr(dv_dqd,dv_dqd2,DEBUG_MODE)
    print("da/dq")
    printErr(da_dq,da_dq2,DEBUG_MODE)
    print("da/dqd")
    printErr(da_dqd,da_dqd2,DEBUG_MODE)
    print("df/dqfp")
    printErr(df_fp_dq,df_fp_dq2,DEBUG_MODE)
    print("df/dqdfp")
    printErr(df_fp_dqd,df_fp_dqd2,DEBUG_MODE)
    print("df/dq")
    printErr(df_dq,df_dq2,DEBUG_MODE)
    print("df/dqd")
    printErr(df_dqd,df_dqd2,DEBUG_MODE)
    print("dc/dq")
    printErr(dc_dq,dc_dq2,DEBUG_MODE)
    print("dc/dqd")
    printErr(dc_dqd,dc_dqd2,DEBUG_MODE)

if __name__ == "__main__":
    main()