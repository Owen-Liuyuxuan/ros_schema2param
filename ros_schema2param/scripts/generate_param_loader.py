#!/usr/bin/env python3
"""
Generate C++ parameter loader from JSON Schema.
Supports: bool, int, double, string, and arrays of these types.
"""

import argparse
import json
import re
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from jinja2 import Environment, FileSystemLoader
import jsonschema


@dataclass
class ParameterInfo:
    """Information about a single parameter."""
    name: str
    cpp_name: str
    schema_type: str  # JSON schema type
    cpp_type: str
    rclcpp_getter: str  # as_bool(), as_int(), etc.
    default: Any
    description: str
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    enum_values: Optional[List[str]] = None
    is_array: bool = False
    array_item_type: Optional[str] = None
    is_nested: bool = False
    nested_params: Optional[List['ParameterInfo']] = None


class SchemaParser:
    """Parse JSON Schema and extract parameter information."""
    
    # Mapping from JSON Schema types to C++ types and ROS2 getters
    TYPE_MAPPING = {
        'boolean': ('bool', 'as_bool'),
        'integer': ('int64_t', 'as_int'),
        'number': ('double', 'as_double'),
        'string': ('std::string', 'as_string'),
    }
    
    ARRAY_TYPE_MAPPING = {
        'boolean': ('std::vector<bool>', 'as_bool_array'),
        'integer': ('std::vector<int64_t>', 'as_integer_array'),
        'number': ('std::vector<double>', 'as_double_array'),
        'string': ('std::vector<std::string>', 'as_string_array'),
    }
    
    def __init__(self, schema_path: Path):
        with open(schema_path, 'r') as f:
            self.schema = json.load(f)
        
        # Validate schema
        try:
            jsonschema.Draft7Validator.check_schema(self.schema)
        except jsonschema.SchemaError as e:
            raise ValueError(f"Invalid JSON Schema: {e}")
    
    def to_cpp_name(self, name: str) -> str:
        """Convert parameter name to C++ variable name."""
        # Remove special characters and convert to snake_case
        cpp_name = re.sub(r'[^\w]', '_', name)
        return cpp_name
    
    def parse(self) -> Tuple[List[ParameterInfo], str]:
        """Parse schema and return list of parameters and namespace."""
        # Navigate to the actual parameters definition
        definitions = self.schema.get('definitions', {})
        
        # Find the main parameter definition
        main_def_name = None
        for def_name, def_schema in definitions.items():
            if def_schema.get('type') == 'object':
                main_def_name = def_name
                break
        
        if not main_def_name:
            raise ValueError("Could not find main parameter definition in schema")
        
        main_def = definitions[main_def_name]
        properties = main_def.get('properties', {})
        
        params = self._parse_properties(properties)
        namespace = self.get_namespace()
        
        return params, namespace
    
    def _parse_properties(self, properties: Dict, prefix: str = '') -> List[ParameterInfo]:
        """Recursively parse properties."""
        params = []
        
        for name, prop in properties.items():
            full_name = f"{prefix}.{name}" if prefix else name
            cpp_name = self.to_cpp_name(name)
            
            param_type = prop.get('type')
            
            # Handle null type (skip)
            if param_type == 'null':
                continue
            
            # Handle arrays
            if param_type == 'array':
                param = self._parse_array_param(full_name, cpp_name, prop)
                params.append(param)
            
            # Handle nested objects
            elif param_type == 'object':
                nested_props = prop.get('properties', {})
                nested_params = self._parse_properties(nested_props, full_name)
                
                # Create struct name from parameter name
                struct_name = ''.join(word.capitalize() for word in cpp_name.split('_'))
                struct_name += 'Params'
                
                param = ParameterInfo(
                    name=full_name,
                    cpp_name=cpp_name,
                    schema_type='object',
                    cpp_type=struct_name,
                    rclcpp_getter='',
                    default=None,
                    description=prop.get('description', ''),
                    is_nested=True,
                    nested_params=nested_params
                )
                params.append(param)
            
            # Handle primitive types
            else:
                param = self._parse_primitive_param(full_name, cpp_name, prop)
                params.append(param)
        return params
    
    def _parse_primitive_param(
        self, 
        full_name: str, 
        cpp_name: str, 
        prop: Dict
    ) -> ParameterInfo:
        """Parse a primitive type parameter."""
        param_type = prop.get('type')
        
        if param_type not in self.TYPE_MAPPING:
            raise ValueError(f"Unsupported parameter type: {param_type}")
        
        cpp_type, rclcpp_getter = self.TYPE_MAPPING[param_type]
        
        return ParameterInfo(
            name=full_name,
            cpp_name=cpp_name,
            schema_type=param_type,
            cpp_type=cpp_type,
            rclcpp_getter=rclcpp_getter,
            default=prop.get('default'),
            description=prop.get('description', ''),
            minimum=prop.get('minimum'),
            maximum=prop.get('maximum'),
            enum_values=prop.get('enum')
        )
    
    def _parse_array_param(
        self, 
        full_name: str, 
        cpp_name: str, 
        prop: Dict
    ) -> ParameterInfo:
        """Parse an array type parameter."""
        items = prop.get('items', {})
        item_type = items.get('type')
        
        if item_type not in self.ARRAY_TYPE_MAPPING:
            raise ValueError(f"Unsupported array item type: {item_type}")
        
        cpp_type, rclcpp_getter = self.ARRAY_TYPE_MAPPING[item_type]
        
        return ParameterInfo(
            name=full_name,
            cpp_name=cpp_name,
            schema_type='array',
            cpp_type=cpp_type,
            rclcpp_getter=rclcpp_getter,
            default=prop.get('default'),
            description=prop.get('description', ''),
            is_array=True,
            array_item_type=item_type,
            minimum=items.get('minimum'),
            maximum=items.get('maximum')
        )
    
    def get_namespace(self) -> str:
        """Extract namespace from schema title."""
        title = self.schema.get('title', 'parameters')
        # Convert "Parameters for dummy_perception_publisher_node" to namespace
        match = re.search(r'for\s+(\w+)', title)
        if match:
            return match.group(1)
        return 'parameters'


class CodeGenerator:
    """Generate C++ code from parameter information."""
    
    def __init__(self, template_dir: Path):
        self.env = Environment(
            loader=FileSystemLoader(str(template_dir)),
            trim_blocks=True,
            lstrip_blocks=True
        )
        
        # Add custom filters
        self.env.filters['cpp_default'] = self.cpp_default_value
        self.env.filters['cpp_array_default'] = self.cpp_array_default_value
    
    def cpp_default_value(self, value: Any, cpp_type: str) -> str:
        """Convert Python default value to C++ literal."""
        if value is None:
            return ''
        
        if cpp_type == 'std::string':
            return f'"{value}"'
        elif cpp_type == 'bool':
            return 'true' if value else 'false'
        elif cpp_type == 'double':
            return f'{value}'
        elif cpp_type == 'int64_t':
            return f'{value}L'
        else:
            return str(value)
    
    def cpp_array_default_value(self, value: List[Any], item_type: str) -> str:
        """Convert Python list to C++ vector initializer."""
        if not value:
            return '{}'
        
        if item_type == 'string':
            items = [f'"{v}"' for v in value]
        elif item_type == 'boolean':
            items = ['true' if v else 'false' for v in value]
        elif item_type == 'number':
            items = [str(v) for v in value]
        elif item_type == 'integer':
            items = [f'{v}L' for v in value]
        else:
            items = [str(v) for v in value]
        
        return '{' + ', '.join(items) + '}'
    
    def generate_header(
        self,
        params: List[ParameterInfo],
        namespace: str,
        class_name: str,
        header_only: bool = False
    ) -> str:
        """Generate header file."""
        template = self.env.get_template('param_loader.hpp.j2')
        
        nested_structs = [p for p in params if p.is_nested]
        flat_params = [p for p in params]
        
        return template.render(
            namespace=namespace,
            class_name=class_name,
            params=flat_params,
            nested_structs=nested_structs,
            header_only=header_only,
            timestamp=__import__('datetime').datetime.now().isoformat()
        )
    
    def generate_source(
        self,
        params: List[ParameterInfo],
        namespace: str,
        class_name: str
    ) -> str:
        """Generate source file."""
        template = self.env.get_template('param_loader.cpp.j2')
        
        nested_structs = [p for p in params if p.is_nested]
        flat_params = [p for p in params]
        
        return template.render(
            namespace=namespace,
            class_name=class_name,
            params=flat_params,
            nested_structs=nested_structs,
            timestamp=__import__('datetime').datetime.now().isoformat()
        )


def main():
    parser = argparse.ArgumentParser(
        description='Generate C++ parameter loader from JSON Schema'
    )
    parser.add_argument('--schema', required=True, help='JSON Schema file')
    parser.add_argument('--output-dir', required=True, help='Output directory')
    parser.add_argument('--template-dir', required=True, help='Template directory')
    parser.add_argument('--class-name', default='ParameterLoader', help='Class name')
    parser.add_argument('--namespace', help='C++ namespace (auto-detected if not provided)')
    parser.add_argument('--header-only', action='store_true', help='Generate header-only')
    
    args = parser.parse_args()
    
    schema_path = Path(args.schema)
    output_dir = Path(args.output_dir)
    template_dir = Path(args.template_dir)
    
    # Parse schema
    print(f"Parsing schema: {schema_path}")
    schema_parser = SchemaParser(schema_path)
    params, detected_namespace = schema_parser.parse()
    
    namespace = args.namespace or detected_namespace
    
    print(f"Found {len(params)} parameters")
    print(f"Namespace: {namespace}")
    
    # Generate code
    generator = CodeGenerator(template_dir)
    
    # Generate header
    header_content = generator.generate_header(
        params, namespace, args.class_name, args.header_only
    )
    header_path = output_dir / f"{args.class_name}.hpp"
    with open(header_path, 'w') as f:
        f.write(header_content)
    print(f"Generated: {header_path}")
    
    # Generate source (if not header-only)
    if not args.header_only:
        source_content = generator.generate_source(
            params, namespace, args.class_name
        )
        source_path = output_dir / f"{args.class_name}.cpp"
        with open(source_path, 'w') as f:
            f.write(source_content)
        print(f"Generated: {source_path}")
    
    print("Code generation completed successfully!")


if __name__ == '__main__':
    main()
