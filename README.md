# 了解 catkin_make,catkin build,cmake, make,g++

## g++:

    test_1.cpp

```cpp
#include <iostream>

int main(int argc, char **argv)
{
    printf("test_1 g++\r\n");
    printf("test_1 g++\r\n");
    printf("test_1 g++\r\n");
    printf("test_1 g++\r\n");
    return 0;
}

```

    命令行：

```
g++ -o t1 test_1.cpp 
./t1
```

## make:

include/myheader.h

```c++
#ifndef MYHEADER_H
#define MYHEADER_H

// 这里可以放一些声明

#endif // MYHEADER_H
```

src/main.cpp

```c++
#include <iostream>
#include "myheader.h"

int main() {
    std::cout << "Hello, Make!" << std::endl;
    return 0;
}

```

Makefile

```makefile
# 编译器
CXX = g++
# 编译器选项
CXXFLAGS = -std=c++11 -I include
# 源文件目录
SRCDIR = src
# 目标文件目录
OBJDIR = obj
# 可执行文件名称
TARGET = MyProject
# 查找源文件
SOURCES = $(wildcard $(SRCDIR)/*.cpp)
# 生成目标文件列表
OBJECTS = $(patsubst $(SRCDIR)/%.cpp, $(OBJDIR)/%.o, $(SOURCES))
# 默认目标
all: $(TARGET)
# 链接目标文件生成可执行文件
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@
# 编译源文件生成目标文件
$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@
# 创建目标文件目录
$(OBJDIR):
	mkdir -p $(OBJDIR)
# 清理生成的文件
clean:
	rm -rf $(OBJDIR) $(TARGET)
.PHONY: all clean

```

命令行：

```bash
make
make clean
./MyProject
```

## cmake：

src/main.cpp

```c++
#include <iostream>
#include "myheader.h"

int main() {
    std::cout << "Hello, CMake!" << std::endl;
    return 0;
}

```

include/myheader.h

```c++
#ifndef MYHEADER_H
#define MYHEADER_H

// 这里可以放一些声明

#endif // MYHEADER_H

```

CMakeList.txt

```cmake
cmake_minimum_required(VERSION 3.10)
# 设置项目名称
project(MyProject)
# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# 添加包含目录
include_directories(include)
# 查找源文件
file(GLOB SOURCES "src/*.cpp")
# 添加可执行文件
add_executable(MyProject ${SOURCES})
```

命令行

```bash
mkdir build
cd build/
cmake ..
make
./MyProject
```

g++:编译器，用于将 C++ 源代码编译成目标文件或可执行文件

make: 使用 `Makefile` 来自动化调用 `g++` 进行编译和链接

cmake: 生成 `Makefile` 或其他构建系统文件，然后可以使用 `make` 来进行实际的构建过程

# `catkin_make` 和 `catkin build` 的关联与区别：

## 关联：

* 两者都是用于构建 ROS 工作空间的工具，最终目标都是生成可执行文件、库和其他构建产物。
* 都基于 CMake 和 catkin 构建系统。

## 区别：

* **构建空间管理** ：`catkin_make` 使用单一构建空间，而 `catkin build` 为每个包创建独立的构建空间。
* **并行构建** ：`catkin build` 支持并行构建多个包，而 `catkin_make` 只能并行构建单个包内的目标文件。
* **配置灵活性** ：`catkin build` 提供了更多的配置选项和管理工具，如 `catkin config` 和 `catkin profile`。
* **构建输出** ：`catkin build` 提供更详细和可定制的构建输出。

# 自定义消息和common msg
