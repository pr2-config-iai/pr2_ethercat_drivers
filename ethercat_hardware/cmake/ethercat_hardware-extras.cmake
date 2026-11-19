# Our public headers use boost::mutex / boost::thread inline, so consumers need these targets.
find_package(Boost REQUIRED COMPONENTS thread system)
