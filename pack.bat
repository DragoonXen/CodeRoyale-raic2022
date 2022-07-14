"C:\Program Files\Git\git-bash.exe" -c "jar -cfM apack.zip $(find . | egrep '\.(cpp|h|hpp|txt)$' | grep -v ./cmake | grep -v ./csimple | grep -v ./rewind)"
