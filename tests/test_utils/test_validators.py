"""
Tests for utility validator functions.
"""
import pytest
import json
from datetime import datetime
from app.utils.validators import (
    validate_command_string,
    sanitize_command,
    is_valid_json,
    parse_json_safely,
    format_timestamp,
    validate_topic_name,
    validate_node_name,
    truncate_string,
    safe_get_nested
)


@pytest.mark.utils
class TestCommandValidation:
    """Test command validation functions."""
    
    def test_validate_command_string_valid(self):
        """Test validating a valid command string."""
        assert validate_command_string("move_forward") is True
        assert validate_command_string("test command") is True
        assert validate_command_string("a") is True
    
    def test_validate_command_string_empty(self):
        """Test validating an empty command string."""
        assert validate_command_string("") is False
        assert validate_command_string("   ") is False
        assert validate_command_string("\t\n") is False
    
    def test_validate_command_string_too_long(self):
        """Test validating a command string that's too long."""
        long_command = "a" * 10001
        assert validate_command_string(long_command) is False
    
    def test_validate_command_string_max_length_custom(self):
        """Test validating with custom max length."""
        command = "a" * 100
        assert validate_command_string(command, max_length=50) is False
        assert validate_command_string(command, max_length=200) is True
    
    def test_validate_command_string_not_string(self):
        """Test validating a non-string input."""
        assert validate_command_string(123) is False
        assert validate_command_string(None) is False
        assert validate_command_string([]) is False
        assert validate_command_string({}) is False
    
    def test_sanitize_command_normal(self):
        """Test sanitizing a normal command."""
        command = "move_forward"
        result = sanitize_command(command)
        assert result == "move_forward"
    
    def test_sanitize_command_control_characters(self):
        """Test sanitizing command with control characters."""
        command = "move\x00forward\x01test"
        result = sanitize_command(command)
        assert "\x00" not in result
        assert "\x01" not in result
        assert "move" in result
        assert "forward" in result
    
    def test_sanitize_command_whitespace(self):
        """Test sanitizing command with whitespace."""
        command = "  move_forward  "
        result = sanitize_command(command)
        assert result == "move_forward"
    
    def test_sanitize_command_too_long(self):
        """Test sanitizing a command that's too long."""
        long_command = "a" * 20000
        result = sanitize_command(long_command)
        assert len(result) <= 10000
    
    def test_sanitize_command_preserves_newline_tab(self):
        """Test that sanitize preserves newline and tab characters."""
        command = "line1\nline2\tindented"
        result = sanitize_command(command)
        assert "\n" in result
        assert "\t" in result


@pytest.mark.utils
class TestJSONValidation:
    """Test JSON validation functions."""
    
    def test_is_valid_json_valid(self):
        """Test validating valid JSON."""
        assert is_valid_json('{"key": "value"}') is True
        assert is_valid_json('[1, 2, 3]') is True
        assert is_valid_json('{"nested": {"key": "value"}}') is True
    
    def test_is_valid_json_invalid(self):
        """Test validating invalid JSON."""
        assert is_valid_json('{invalid json}') is False
        assert is_valid_json('{"key": "value"') is False
        assert is_valid_json('not json') is False
    
    def test_is_valid_json_empty_string(self):
        """Test validating empty JSON string."""
        assert is_valid_json('') is False
    
    def test_is_valid_json_not_string(self):
        """Test validating non-string input."""
        assert is_valid_json(123) is False
        assert is_valid_json(None) is False
        assert is_valid_json({}) is False
    
    def test_parse_json_safely_valid(self):
        """Test safely parsing valid JSON."""
        json_str = '{"key": "value", "number": 123}'
        result = parse_json_safely(json_str)
        assert result is not None
        assert result["key"] == "value"
        assert result["number"] == 123
    
    def test_parse_json_safely_invalid(self):
        """Test safely parsing invalid JSON."""
        json_str = '{invalid json}'
        result = parse_json_safely(json_str)
        assert result is None
    
    def test_parse_json_safely_empty(self):
        """Test safely parsing empty string."""
        result = parse_json_safely('')
        assert result is None
    
    def test_parse_json_safely_not_string(self):
        """Test safely parsing non-string input."""
        result = parse_json_safely(123)
        assert result is None
        result = parse_json_safely(None)
        assert result is None


@pytest.mark.utils
class TestTimestampFormatting:
    """Test timestamp formatting functions."""
    
    def test_format_timestamp_default(self):
        """Test formatting timestamp with default (now)."""
        result = format_timestamp()
        assert isinstance(result, str)
        # Should be ISO format
        assert 'T' in result or '-' in result
    
    def test_format_timestamp_custom(self):
        """Test formatting a custom timestamp."""
        dt = datetime(2024, 1, 1, 12, 0, 0)
        result = format_timestamp(dt)
        assert isinstance(result, str)
        assert '2024' in result
    
    def test_format_timestamp_none(self):
        """Test formatting timestamp with None (should use now)."""
        result = format_timestamp(None)
        assert isinstance(result, str)


@pytest.mark.utils
class TestROS2Validation:
    """Test ROS2-specific validation functions."""
    
    def test_validate_topic_name_valid(self):
        """Test validating valid ROS2 topic names."""
        assert validate_topic_name("/control/commands") is True
        assert validate_topic_name("/move_group/goal_pose") is True
        assert validate_topic_name("/topic_name") is True
        assert validate_topic_name("/topic/with/multiple/levels") is True
    
    def test_validate_topic_name_invalid(self):
        """Test validating invalid ROS2 topic names."""
        assert validate_topic_name("no_leading_slash") is False
        assert validate_topic_name("/topic with spaces") is False
        assert validate_topic_name("/topic-with-dashes") is False
        assert validate_topic_name("/topic@invalid") is False
    
    def test_validate_topic_name_empty(self):
        """Test validating empty topic name."""
        assert validate_topic_name("") is False
    
    def test_validate_topic_name_not_string(self):
        """Test validating non-string topic name."""
        assert validate_topic_name(123) is False
        assert validate_topic_name(None) is False
    
    def test_validate_node_name_valid(self):
        """Test validating valid ROS2 node names."""
        assert validate_node_name("fastapi_control_node") is True
        assert validate_node_name("node123") is True
        assert validate_node_name("test_node") is True
        assert validate_node_name("NODE_NAME") is True
    
    def test_validate_node_name_invalid(self):
        """Test validating invalid ROS2 node names."""
        assert validate_node_name("node with spaces") is False
        assert validate_node_name("node-with-dashes") is False
        assert validate_node_name("node@invalid") is False
        assert validate_node_name("/node_with_slash") is False
    
    def test_validate_node_name_empty(self):
        """Test validating empty node name."""
        assert validate_node_name("") is False
        assert validate_node_name("   ") is False
    
    def test_validate_node_name_not_string(self):
        """Test validating non-string node name."""
        assert validate_node_name(123) is False
        assert validate_node_name(None) is False


@pytest.mark.utils
class TestStringUtilities:
    """Test string utility functions."""
    
    def test_truncate_string_short(self):
        """Test truncating a short string."""
        result = truncate_string("short", max_length=10)
        assert result == "short"
    
    def test_truncate_string_long(self):
        """Test truncating a long string."""
        long_string = "a" * 100
        result = truncate_string(long_string, max_length=50)
        assert len(result) == 50
        assert result.endswith("...")
    
    def test_truncate_string_exact_length(self):
        """Test truncating string at exact length."""
        string = "a" * 50
        result = truncate_string(string, max_length=50)
        assert result == string
    
    def test_truncate_string_custom_suffix(self):
        """Test truncating with custom suffix."""
        long_string = "a" * 100
        result = truncate_string(long_string, max_length=50, suffix="[truncated]")
        assert len(result) == 50
        assert result.endswith("[truncated]")
    
    def test_safe_get_nested_exists(self):
        """Test getting nested value that exists."""
        data = {"level1": {"level2": {"level3": "value"}}}
        result = safe_get_nested(data, "level1", "level2", "level3")
        assert result == "value"
    
    def test_safe_get_nested_missing(self):
        """Test getting nested value that doesn't exist."""
        data = {"level1": {"level2": {}}}
        result = safe_get_nested(data, "level1", "level2", "level3")
        assert result is None
    
    def test_safe_get_nested_default(self):
        """Test getting nested value with default."""
        data = {"level1": {}}
        result = safe_get_nested(data, "level1", "level2", default="default_value")
        assert result == "default_value"
    
    def test_safe_get_nested_not_dict(self):
        """Test getting nested value when intermediate is not a dict."""
        data = {"level1": "not_a_dict"}
        result = safe_get_nested(data, "level1", "level2")
        assert result is None
    
    def test_safe_get_nested_empty_path(self):
        """Test getting nested value with empty path."""
        data = {"key": "value"}
        result = safe_get_nested(data)
        assert result == data

