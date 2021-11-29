import sys

def signal_handler(sig, frame):
    print('')
    print('bye bye!')
    sys.exit(0)
