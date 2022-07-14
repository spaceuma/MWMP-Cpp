if test -f "build/runUnitTests"; then
    mv build/runUnitTests test/unit/
fi

cd test/unit
if test -f "runUnitTests"; then
    ./runUnitTests
else
    echo "ERROR: The runUnitTests executable was not found in build/ or test/unit/. Compile first the library by running the install.sh script."
    exit
fi
cd ../../
echo "Finished execution of runUnitTests.sh"
