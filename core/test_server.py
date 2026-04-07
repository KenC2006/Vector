#!/usr/bin/env python3
"""
Test script for the JSON-RPC server.

Usage:
    python test_server.py
"""
import subprocess
import json
import sys
import os
import time

def test_server():
    """Test the JSON-RPC server."""
    # Start the server process
    server_process = subprocess.Popen(
        [sys.executable, "server.py"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=os.path.dirname(os.path.abspath(__file__)),
        text=True,
        bufsize=1,
    )

    try:
        # Test 1: ping
        print("Test 1: ping")
        ping_request = {
            "jsonrpc": "2.0",
            "method": "ping",
            "params": {},
            "id": 1,
        }
        server_process.stdin.write(json.dumps(ping_request) + "\n")
        server_process.stdin.flush()

        response = server_process.stdout.readline()
        if response:
            response_data = json.loads(response)
            print(f"  Response: {response_data}")
            if response_data.get("result") == "pong":
                print("  ✓ PASS\n")
            else:
                print("  ✗ FAIL\n")
                return False
        else:
            print("  ✗ No response\n")
            return False

        # Test 2: parse_urdf with sample URDF
        print("Test 2: parse_urdf")
        urdf_path = os.path.join(
            os.path.dirname(__file__),
            "test_data",
            "simple_arm.urdf"
        )

        parse_request = {
            "jsonrpc": "2.0",
            "method": "parse_urdf",
            "params": {
                "path": urdf_path,
            },
            "id": 2,
        }
        server_process.stdin.write(json.dumps(parse_request) + "\n")
        server_process.stdin.flush()

        response = server_process.stdout.readline()
        if response:
            response_data = json.loads(response)
            result = response_data.get("result", {})
            print(f"  Root link: {result.get('root_link')}")
            print(f"  Links: {len(result.get('links', []))}")
            print(f"  Joints: {len(result.get('joints', []))}")
            if result.get("root_link") == "base_link" and len(result.get("links", [])) == 5:
                print("  ✓ PASS\n")
            else:
                print("  ✗ FAIL\n")
                return False
        else:
            print("  ✗ No response\n")
            return False

        # Test 3: method not found
        print("Test 3: method not found")
        bad_request = {
            "jsonrpc": "2.0",
            "method": "nonexistent",
            "params": {},
            "id": 3,
        }
        server_process.stdin.write(json.dumps(bad_request) + "\n")
        server_process.stdin.flush()

        response = server_process.stdout.readline()
        if response:
            response_data = json.loads(response)
            error = response_data.get("error", {})
            if error.get("code") == -32601:
                print(f"  Error: {error.get('message')}")
                print("  ✓ PASS\n")
            else:
                print("  ✗ FAIL\n")
                return False
        else:
            print("  ✗ No response\n")
            return False

        print("All tests passed!")
        return True

    finally:
        # Terminate the server
        server_process.terminate()
        server_process.wait(timeout=2)


if __name__ == "__main__":
    success = test_server()
    sys.exit(0 if success else 1)
