import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alyssa/Documenten/GitHub/Selfdriving-portfolio-2/install/odometry_pkg'
