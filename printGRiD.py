#!/usr/bin/python3
from URDFParser import URDFParser
from GRiDCodeGenerator import GRiDCodeGenerator
from util import parseInputs, printUsage, validateRobot, initializeValues
import subprocess
import sys

def main():
    inputs = parseInputs(NO_ARG_OPTION = True)
    if not inputs is None:
        URDF_PATH, DEBUG_MODE = inputs
        parser = URDFParser()
        robot = parser.parse(URDF_PATH)

        validateRobot(robot, NO_ARG_OPTION = True)

        codegen = GRiDCodeGenerator(robot,DEBUG_MODE,True)
        print("-----------------")
        print("Generating GRiD.cuh")
        print("-----------------")
        codegen.gen_all_code()
        print("New code generated and saved to grid.cuh!")

    print("-----------------")
    print("Compiling printGRiD")
    print("-----------------")
    result = subprocess.run( \
        ["nvcc", "-std=c++11", "-o", "printGRiD.exe", "printGRiD.cu", \
         "-gencode", "arch=compute_86,code=sm_86", \
         "-O3", "-ftz=true", "-prec-div=false", "-prec-sqrt=false"], \
        capture_output=True, text=True \
    )
    if result.stderr:
        print("Compilation errors follow:")
        print(result.stderr)
        exit()

    print("-----------------")
    print("Running printGRiD")
    print("-----------------")
    result = subprocess.run(["./printGRiD.exe"], capture_output=True, text=True)
    if result.stderr:
        print("Runtime errors follow:")
        print(result.stderr)
        exit()

    print(result.stdout)

main()