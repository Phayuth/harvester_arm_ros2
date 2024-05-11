import rtde_control
import rtde_receive


def documation_extract():
    print(help(rtde_control.RTDEControlInterface))
    print(help(rtde_receive.RTDEReceiveInterface))

    file_path = "/home/yuth/ws_yuthdev/ws_rosws/src/harvester_arm_ros2/sssssss"

    try:
        with open(file_path, "r") as file:
            file_contents = file.read()
            print(f"> file_contents: {file_contents}")

    except FileNotFoundError:
        print(f"The file '{file_path}' does not exist.")
    except Exception as e:
        print(f"An error occurred: {e}")

    lines = file_contents.split("\n")
    strings = []
    for line in lines:
        if line.startswith(" "):  # Only include lines starting with spaces (function descriptions)
            strings.append(line.strip())

    from fpdf import FPDF

    pdf = FPDF()
    pdf.add_page()
    pdf.set_font("Arial", "B", 12)
    pdf.cell(40, 10, file_contents)
    pdf.output("tuto1.pdf", "F")
