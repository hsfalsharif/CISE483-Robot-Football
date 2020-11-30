To generate the code for the desired language 
only change the compile.sh file and run it

protoc --[desired language]=[location for generated files] *.proto

Note/
*.proto mean generate the code for all the .proto files in the directory
to generate a specific file only write it insted of *.proto

after that
open the terminal and run which will generate the code in
the specified location:

./compile.sh


