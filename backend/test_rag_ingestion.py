import unittest
from unittest.mock import Mock, patch, MagicMock
import os
from main import is_valid_url, normalize_url, sanitize_content, validate_embeddings, chunk_text, validate_configuration
from main import get_all_urls, extract_text_from_url, embed, create_collection, save_chunk_to_qdrant


class TestURLUtilities(unittest.TestCase):
    def test_is_valid_url_valid(self):
        """Test that valid URLs are recognized as valid"""
        valid_urls = [
            "https://example.com",
            "http://example.com",
            "https://subdomain.example.com/path",
            "https://example.com:8080/path?query=value"
        ]

        for url in valid_urls:
            with self.subTest(url=url):
                self.assertTrue(is_valid_url(url))

    def test_is_valid_url_invalid(self):
        """Test that invalid URLs are recognized as invalid"""
        invalid_urls = [
            "not-a-url",
            "http://",
            "",
            "justtext",
            "ftp://example.com"  # Though technically valid, may not be web accessible
        ]

        for url in invalid_urls:
            with self.subTest(url=url):
                self.assertFalse(is_valid_url(url))

    def test_normalize_url_adds_scheme(self):
        """Test that normalize_url adds https scheme when missing"""
        result = normalize_url("example.com")
        self.assertEqual(result, "https://example.com")

    def test_normalize_url_removes_trailing_slash(self):
        """Test that normalize_url removes trailing slashes"""
        result = normalize_url("https://example.com/")
        self.assertEqual(result, "https://example.com")

        result = normalize_url("https://example.com/path/")
        self.assertEqual(result, "https://example.com/path")

    def test_normalize_url_preserves_existing_scheme(self):
        """Test that normalize_url preserves existing schemes"""
        result = normalize_url("http://example.com/")
        self.assertEqual(result, "http://example.com")


class TestContentProcessing(unittest.TestCase):
    def test_sanitize_content_removes_script_tags(self):
        """Test that sanitize_content removes script tags"""
        content_with_script = "<p>Text</p><script>alert('xss')</script><p>More text</p>"
        expected = "<p> Text </p> <p> More text </p>"
        result = sanitize_content(content_with_script)
        self.assertEqual(result.strip(), expected.strip())

    def test_sanitize_content_removes_style_tags(self):
        """Test that sanitize_content removes style tags"""
        content_with_style = "<p>Text</p><style>body { color: red; }</style><p>More text</p>"
        expected = "<p> Text </p> <p> More text </p>"
        result = sanitize_content(content_with_style)
        self.assertEqual(result.strip(), expected.strip())

    def test_sanitize_content_removes_event_handlers(self):
        """Test that sanitize_content removes event handlers"""
        content_with_handler = '<p onclick="alert()">Text</p>'
        result = sanitize_content(content_with_handler)
        # The event handler should be removed
        self.assertNotIn("onclick", result)

    def test_sanitize_content_preserves_safe_content(self):
        """Test that sanitize_content preserves safe content"""
        safe_content = "<p>This is safe content</p><div>More safe content</div>"
        result = sanitize_content(safe_content)
        self.assertIn("safe content", result)

    def test_chunk_text_basic(self):
        """Test basic text chunking functionality"""
        content = "This is a test content that will be chunked into smaller pieces."
        chunks = chunk_text(content, chunk_size=20, overlap=5)

        self.assertGreater(len(chunks), 0)
        for chunk in chunks:
            self.assertIn("content", chunk)
            self.assertIn("start_pos", chunk)
            self.assertIn("end_pos", chunk)

    def test_chunk_text_with_empty_content(self):
        """Test chunk_text with empty content"""
        chunks = chunk_text("")
        self.assertEqual(chunks, [])

    def test_validate_embeddings_correct_dimension(self):
        """Test that validate_embeddings returns True for correct dimensions"""
        embeddings = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
        result = validate_embeddings(embeddings, expected_dimension=3)
        self.assertTrue(result)

    def test_validate_embeddings_incorrect_dimension(self):
        """Test that validate_embeddings returns False for incorrect dimensions"""
        embeddings = [[1.0, 2.0], [4.0, 5.0, 6.0]]  # Mixed dimensions
        result = validate_embeddings(embeddings, expected_dimension=3)
        self.assertFalse(result)

    def test_validate_embeddings_empty_list(self):
        """Test that validate_embeddings returns True for empty list"""
        result = validate_embeddings([])
        self.assertTrue(result)


class TestConfigurationValidation(unittest.TestCase):
    @patch.dict(os.environ, {
        'COHERE_API_KEY': 'test-key',
        'QDRANT_URL': 'https://test.us-east4-0.gcp.cloud.qdrant.io:6333',
        'QDRANT_API_KEY': 'test-api-key'
    })
    def test_validate_configuration_with_all_vars(self):
        """Test that validate_configuration passes when all required vars are present"""
        # This should not raise an exception
        validate_configuration()

    @patch.dict(os.environ, {}, clear=True)
    def test_validate_configuration_missing_vars(self):
        """Test that validate_configuration raises error when required vars are missing"""
        with self.assertRaises(ValueError) as context:
            validate_configuration()

        self.assertIn("Missing required environment variables", str(context.exception))


if __name__ == '__main__':
    unittest.main()