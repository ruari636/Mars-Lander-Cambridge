CC = g++
CCSW = -O3 -Wno-deprecated-declarations
PLATFORM = `uname`
SRC_LOCATION := source
INC_LOCATION := include
OBJ_LOCATION := outputs

LANDER_SRC_FILES := $(shell find $(SRC_LOCATION) -name "*.cpp")
LANDER_INC_FILES := $(shell find $(SRC_LOCATION) -name "*.h")
LANDER_OBJ_FILES := $(patsubst $(SRC_LOCATION)/%.cpp,$(OBJ_LOCATION)/%.o,$(LANDER_SRC_FILES))

all:	lander

lander: $(LANDER_OBJ_FILES)
	@if [ "${PLATFORM}" = "Linux" ]; \
	then \
		$(CC) -o lander $(LANDER_OBJ_FILES) ${CCSW} -lGL -lGLU -lglut; \
		echo Build Complete; \
		echo Linking for Linux; \
	else \
		echo "Unable to make outside of Linux"; \
	fi

$(LANDER_OBJ_FILES): $(LANDER_INC_FILES)

$(OBJ_LOCATION)/%.o: $(SRC_LOCATION)/%.cpp | $(OBJ_LOCATION)  # Rule for creating object files
	@echo building $<
	$(CC) ${CCSW} -I $(INC_LOCATION) -c $< -o $@

$(OBJ_LOCATION):
	mkdir -p $(OBJ_LOCATION)

.cpp.o:
	$(CC) ${CCSW} -c $<

clean:
	@echo cleaning up; /bin/rm -f lander -r outputs

.PHONY: all clean  # Declare targets as phony to ensure they are always executed