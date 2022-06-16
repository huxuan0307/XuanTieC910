
import sys
import re

def usage():
    print("Usage: ")
    print("    %s .v file, config file" % sys.argv[0])
    print("Example:")
    print("    python3 %s 1.v, lsu_idu.txt" % sys.argv[0])

def snake2camel(name:str):
    name_list = name.lstrip("_").split("_")
    res = name_list[0] + ''.join(elem.title() for elem in name_list[1:])
    return res

re_io = {
    "input" : r"input\s+\[([0-9]+)\s*:\s*([0-9]+)\]\s*(\w+)\s*;",
    "output": r"output\s+\[([0-9]+)\s*:\s*([0-9]+)\]\s*(\w+)\s*;"
}
re_io_no_width = {
    "input" : r"input\s+(\w+)\s*;",
    "output": r"output\s+(\w+)\s*;"
}

def get_bundle_class(lines, prefixes:list, io:str, camel = True):
    if io not in ["input", "output"]:
        print("get_class_bundle's arg io must in ['input', 'output']")
        exit()
    bundle_class = dict()
    for prefix in prefixes:
        bundle_class[prefix] = []
    for line in lines:
        res = re.findall(re_io[io], line)
        width = 1
        signal = ""
        selected = False
        if len(res) != 0:
            width = int(res[0][0]) - int(res[0][1]) + 1
            signal = res[0][2]
            selected = True
        else:
            res = re.findall(re_io_no_width[io], line)
            if len(res) != 0:
                selected = True
                signal = res[0]
        if (selected):
            for prefix in prefixes:
                if signal.find(prefix) != -1:
                    bundle_class[prefix].append((snake2camel(signal[len(prefix):]), width))
                    break
    return bundle_class

def get_prefixes_class_names(line):
    prefix_re = "(\w+)"
    prefixes = []
    class_names = []
    field_names = []
    top_class = ""
    for line in prefix_lines:
        res = re.findall(prefix_re, line)
        prefixes.append(res[0])
        class_names.append(res[1])
        field_names.append(res[2])
        if "top_class" == res[2]:
            top_class = res[0]
    return (prefixes, class_names, field_names, top_class)

def generate_scala(bundle_class: dict, class_names: list, field_names: list, need_top=True, top_class="", file=sys.stdout, extends=[]):
    class_head_template = r"class %s extends Bundle %s {"
    class_tail_template = r"}"
    field_template = r"  val %s : %s = new %s"
    value_def_template = r"  val %s : UInt = UInt(%d.W)"
    extends_str = "".join(" with"+elem for elem in extends)
    top_idx = -1
    for i, item in enumerate(bundle_class.items()):
        if need_top and item[0] == top_class:
            top_idx = i
            continue
        print(class_head_template % (class_names[i], extends_str), file = file)
        for signal, width in item[1]:
            print(value_def_template % (signal, width), file = file)
        print(class_tail_template, file = file)
        print(file = file)
    if need_top:
        print(class_head_template % (class_names[top_idx], extends_str), file = file)
        for i, field in enumerate(field_names):
            if field == "top_class":
                continue
            print(field_template % (field, class_names[i], class_names[i]), file = file)
        for signal, width in bundle_class[top_class]:
            print(value_def_template % (signal, width), file = file)
        print(class_tail_template, file = file)
        print(file = file)

if len(sys.argv) < 3:
    usage()
    exit()

prefix_file_obj = open(sys.argv[2], "r")
direction = prefix_file_obj.readline().strip()
prefix_lines = prefix_file_obj.readlines()
# print("direction: ", direction)
prefix_file_obj.close()
prefixes, class_names, field_names, top_class = get_prefixes_class_names(prefix_lines)
need_top = top_class != ""

file = sys.argv[1]
file_name, _ = file.split(".")
out_file = file_name + ".scala"
fobj = open(file, "r")

lines = fobj.readlines()
fobj.close()

bundle_class = get_bundle_class(lines, prefixes, direction, camel = True)


generate_scala(bundle_class, class_names, field_names, file=open(out_file, "w"), need_top=need_top, top_class=top_class)




