# Compiler and flags
CXX := g++
CXXFLAGS := -Isrc/Include
LDFLAGS := -Lsrc/lib
LIBS := -lmingw32 -lSDL2main -lSDL2 -lSDL2_ttf

# Target executable
TARGET := main
SRC := warehouse.cc

# Build rule
$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $(TARGET) $(SRC) $(LIBS)

# Clean rule
clean:
	rm -f $(TARGET)
