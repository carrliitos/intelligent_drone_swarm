<?php
if ($_SERVER["REQUEST_METHOD"] == "POST") {
    // Using absolute path, modify as needed
    $script_path = "/Users/griffinpolly/Downloads/esp32_drone_swarm/src/main.py";
    $venv_path = "/Users/griffinpolly/Downloads/esp32_drone_swarm/venv/bin/activate";

    // If the start button is pressed
    if (isset($_POST["start_script"])) {
        // Run the script inside the virtual environment
        $command = "/bin/bash -c 'source $venv_path && python3 $script_path 2>&1'";
        $output = shell_exec($command);
        // Return the output to the browser
        header('Content-Type: text/plain');
        echo $output;
        exit();
    }

    // If the stop button is pressed
    //**NOT WORKING: logs do not show if stop button is pressed, and drone does not stop :(
    if (isset($_POST["stop_script"])) {
        //mimick cntrl + c
        shell_exec("pkill -2 -f " . escapeshellarg($script_path));
        header('Content-Type: text/plain');
        echo "Script stopped.";
        exit();
    }
}
?>
