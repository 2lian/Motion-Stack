from pydantic.json_schema import model_json_schema
import json
import os
import sys
from pathlib import Path

# Add the package root to sys.path
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(package_root)

from environments.config import SimConfigToml

if __name__ == "__main__":
    # Generate schema for SimConfig
    schema = model_json_schema(
        SimConfigToml,
        # title="Simulation Configuration Schema",
        # description="JSON schema for moonbot simulation environment configuration",
    )

    # Save next to this script as schema.json
    schema_path = Path(__file__).parent / "schema.json"

    # Save the schema to file
    with open(schema_path, "w") as f:
        # Add header
        f.write("// Generated schema for SimConfig\n")
        f.write("// To regenerate run python3 src/sim/moonbot_isaac/config/generate_schema.py\n\n")
        json.dump(schema, fp=f)

    print(f"Schema successfully generated and saved to: {schema_path}")
