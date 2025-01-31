# Boost SML Vendor Package
For usage and reference: https://github.com/boost-ext/sml

## Create UML Diagram from your code
If you execute the following C++ file (replace the required parts), a text file will be generated with plant-uml syntax.

```cpp
#include <boost/algorithm/string/replace.hpp>
#include <fstream>
#include <iostream>
#include <regex>
#include <sstream>

#include "boost/sml/utility/dump_plant_uml.hpp"
#include "**YOUR_STATE_MACHINE**.hpp"
int main()
{
  // Get Plant UML String
  std::stringstream ss;
  dump<**YOUR_STATE_MACHINE**>(ss);
  std::string data = ss.str();

  // Remove namespace
  boost::replace_all(data, "tam::sm::", "");

  // save to file
  std::ofstream ostrm("state_machine.puml");
  ostrm << data;
  ostrm.close();

  return 0;
}
// Self Transition not working
```

To create a PNG from this text file you need to install the plantuml library (`sudo apt-get install plantuml`). Below there is a bash-script (replace the required parts) which will do the whole process at once.
You can execute it e.g. in a Pre-Commit Hook within your repository, to have the current State-Machine Graph in your README.md.

```bash
#!/bin/bash
/bin/bash -c "colcon build --packages-up-to **YOUR_PACKAGE** --base-paths .. && . install/setup.bash && ros2 run **THE_CPP_FILE_ABOVE**"
if [ $? -eq 1 ]
    then
        echo "Build failed!"
        rm -rf build install log
        exit 1
fi
rm -rf build install log
export PLANTUML_LIMIT_SIZE=8192
plantuml -tpng state_machine.puml
echo "New Files generated!"
exit 0
```