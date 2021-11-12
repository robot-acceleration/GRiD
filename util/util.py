import sys
import pathlib
import random
import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth = 100)

def printUsage(NO_ARG_OPTION = False):
    print("Usage is: script.py PATH_TO_URDF (FILE_NAMESPACE_NAME) (-D)")
    print("                    where -D indicates full debug mode")
    if NO_ARG_OPTION:
        print("Alternative usage assuming grid.cuh is already generated: script.py")

def fileExists(FILE_PATH):
    return pathlib.Path(FILE_PATH).is_file()

def validateFile(FILE_PATH, NO_ARG_OPTION = False):
    if not fileExists(FILE_PATH):
        print("[!Error] grid.cuh does not exist")
        printUsage(NO_ARG_OPTION)
        exit()

def parseInputs(NO_ARG_OPTION = False):
    args = sys.argv[1:]
    if len(args) == 0:
        if NO_ARG_OPTION:
            validateFile("grid.cuh", NO_ARG_OPTION)
            print("Using generated grid.cuh")
            return None
        print("[!Error] No URDF filepath specified")
        printUsage(NO_ARG_OPTION)
        exit()
    
    URDF_PATH = args[0]
    validateFile(URDF_PATH, NO_ARG_OPTION)

    DEBUG_MODE_1 = True if len(args) > 1 and ((args[1] == "-D") or (args[1] == "-d")) else False
    DEBUG_MODE_2 = True if len(args) > 2 and ((args[2] == "-D") or (args[2] == "-d")) else False
    DEBUG_MODE = DEBUG_MODE_1 or DEBUG_MODE_2

    if len(args) > 2:
        FILE_NAMESPACE_NAME = args[1] if DEBUG_MODE_2 else args[2]
    elif len(args) > 1 and not DEBUG_MODE_1:
        FILE_NAMESPACE_NAME = args[1]
    else:
        FILE_NAMESPACE_NAME = "grid"

    print("Running with: DEBUG_MODE = " + str(DEBUG_MODE))
    print("                    URDF = " + URDF_PATH)
    print("                    NAME = " + FILE_NAMESPACE_NAME)

    return (URDF_PATH, DEBUG_MODE, FILE_NAMESPACE_NAME)

def validateRobot(robot, NO_ARG_OPTION = False):
    if robot == None:
        print("[!Error] URDF parsing failed. Please make sure you input a valid URDF file.")
        printUsage(NO_ARG_OPTION)
        exit()

def printErr(a, b, FULL_DEBUG = False, TOLERANCE = 1e-10):
    err = a - b
    err = abs(err) > TOLERANCE
    if err.any():
        print(err)
        if (FULL_DEBUG):
            print("Inputs were:")
            print(a)
            print(b)
    else:
        print("  passed")

def initializeValues(robot, MATCH_CPP_RANDOM = False):
    # allocate memory
    n = robot.get_num_pos()
    q = np.zeros((n))
    qd = np.zeros((n))
    u = np.zeros((n))

    if (MATCH_CPP_RANDOM):
        # load CPP rand point
        if n > 0:
            q[0] = -0.336899
            qd[0] = 0.43302
            u[0] = 0.741788
        if n > 1:
            q[1] = 1.29662
            qd[1] = -0.421561
            u[1] = 1.92844
        if n > 2:
            q[2] = -0.677475 
            qd[2] = -0.645439
            u[2] = -0.903882
        if n > 3:
            q[3] = -1.42182
            qd[3] = -1.86055
            u[3] = 0.0333959
        if n > 4:
            q[4] = -0.706676
            qd[4] = -0.0130938
            u[4] = 1.17986
        if n > 5:
            q[5] = -0.134981 
            qd[5] = -0.458284
            u[5] = -1.94599
        if n > 6:
            q[6] = -1.14953
            qd[6] = 0.741174
            u[6] = 0.32869
        if n > 7:
            q[7] = -0.296646
            qd[7] = 1.76642
            u[7] = -0.139457
        if n > 8:
            q[8] = 2.13845
            qd[8] = 0.898011
            u[8] = 2.00667
        if n > 9:
            q[9] = 2.00956
            qd[9] = -1.85675
            u[9] = -0.519292
        if n > 10:
            q[10] = 1.55163
            qd[10] = 1.62223
            u[10] = -0.711198
        if n > 11:
            q[11] = 2.2893
            qd[11] = 0.709379
            u[11] = 0.376638
        if n > 12:
            q[12] = 0.0418005
            qd[12] = -0.382885
            u[12] = -0.209225
        if n > 13:
            q[13] = -0.125271
            qd[13] = -0.239602
            u[13] = -0.816928
        if n > 14:
            q[14] = -1.35512
            qd[14] = 1.88499
            u[14] = -0.943019
        if n > 15:
            q[15] = -0.606463
            qd[15] = -2.20784
            u[15] = -2.16433
        if n > 16:
            q[16] = -2.13552
            qd[16] = -0.921183
            u[16] = 1.37954
        if n > 17:
            q[17] = 0.229695
            qd[17] = -0.110463
            u[17] = 0.456738
        if n > 18:
            q[18] = 0.229592
            qd[18] = -1.64542
            u[18] = -0.702506
        if n > 19:
            q[19] = -0.197398
            qd[19] = -1.7481
            u[19] = 0.159814
        if n > 20:
            q[20] = -0.221438
            qd[20] = -0.562579
            u[20] = 0.944469
        if n > 21:
            q[21] = 1.02441
            qd[21] = 1.02289
            u[21] = 0.100297
        if n > 22:
            q[22] = -0.9309
            qd[22] = 0.21233
            u[22] = -0.1311
        if n > 23:
            q[23] = 1.12961
            qd[23] = 1.30624
            u[23] = 0.750389
        if n > 24:
            q[24] = 0.864741
            qd[24] = 1.31059
            u[24] = -0.666778
        if n > 25:
            q[25] = 0.705222
            qd[25] = -0.0383565
            u[25] = 0.486885
        if n > 26:
            q[26] = 0.0810176
            qd[26] = 0.317353
            u[26] = 0.513445
        if n > 27:
            q[27] = 0.541962
            qd[27] = 0.479234
            u[27] = 0.0573834
        if n > 28:
            q[28] = 1.01213
            qd[28] = 0.55686
            u[28] = 0.425883
        if n > 29:
            q[29] = 2.213
            qd[29] = 0.541122
            u[29] = 0.293804
        if n > 30:
            print("[!ERROR] CPP Random Match only implemented up to n = 30. Please use a lower dof URDF.")
            exit()
    else:
        for i in range(n):
            q[i] = random.random()
            qd[i] = random.random()
            u[i] = random.random()
        
    return q, qd, u, n