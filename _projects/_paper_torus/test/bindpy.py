import sys

sys.path.append("../build/")
import my_module

result = my_module.add(5, 3)
print(f"The result is: {result}")
