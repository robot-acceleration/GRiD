#!/usr/bin/python3
from .URDFParser import URDFParser
from .RBDReference import RBDReference
from .GRiDCodeGenerator import GRiDCodeGenerator
from .util import parseInputs, printUsage, validateRobot, initializeValues, printErr
import numpy as np

def main():
    URDF_PATH, DEBUG_MODE = parseInputs()

    parser = URDFParser()
    robot = parser.parse(URDF_PATH)

    validateRobot(robot)

    reference = RBDReference(robot)
    q, qd, u, n = initializeValues(robot, MATCH_CPP_RANDOM = True)

    print("q")
    print(q)
    print("qd")
    print(qd)
    print("u")
    print(u)

    (c, v, a, f) = reference.rnea(q,qd)
    print("c")
    print(c)

    print("Minv")
    Minv = reference.minv(q)
    print(Minv)

    print("qdd")
    qdd = np.matmul(Minv,(u-c))
    print(qdd)

    dc_du = reference.rnea_grad(q, qd, qdd)
    print("dc/dq with qdd")
    print(dc_du[:,:n])
    print("dc/dqd with qdd")
    print(dc_du[:,n:])

    df_du = np.matmul(-Minv,dc_du)
    print("df/dq")
    print(df_du[:,:n])
    print("df/dqd")
    print(df_du[:,n:])

    if DEBUG_MODE:
        print("-------------------")
        print("printing intermediate outputs from refactorings")
        print("-------------------")
        codegen = GRiDCodeGenerator(robot, DEBUG_MODE)
        (c, v, a, f) = codegen.test_rnea(q,qd)
        print("v")
        print(v)
        print("a")
        print(a)
        print("f")
        print(f)
        print("c")
        print(c)
        
        Minv = codegen.test_minv(q)
        print("Minv")
        print(Minv)

        print("u-c")
        umc = u-c
        print(umc)
        print("qdd")
        qdd = np.matmul(Minv,umc)
        print(qdd)
        
        dc_du = codegen.test_rnea_grad(q, qd, qdd)
        print("dc/dq with qdd")
        print(dc_du[:,:n])
        print("dc/dqd with qdd")
        print(dc_du[:,n:])
        
        df_du = np.matmul(-Minv,dc_du)
        print("df/dq")
        print(df_du[:,:n])
        print("df/dqd")
        print(df_du[:,n:])

if __name__ == "__main__":
    main()