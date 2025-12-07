#!/usr/bin/env node

/**
 * Content Validation Script for Physical AI & Humanoid Robotics Textbook
 * Validates content against constitution requirements:
 * - Code blocks ≤20 lines
 * - ≤2 images per chapter
 * - Proper file formats
 */

const fs = require('fs');
const path = require('path');

// Configuration
const DOCS_DIR = './docs';
const MAX_CODE_LINES = 20;
const MAX_IMAGES_PER_CHAPTER = 2;
const ALLOWED_CODE_LANGUAGES = ['python', 'bash', 'yaml', 'json'];

/**
 * Check if a code block exceeds the line limit
 * @param {string} codeBlock - The code block content
 * @returns {boolean} - True if code block exceeds limit
 */
function isCodeBlockTooLong(codeBlock) {
  const lines = codeBlock.split('\n');
  return lines.length > MAX_CODE_LINES;
}

/**
 * Count images in content
 * @param {string} content - The content to analyze
 * @returns {number} - Number of images found
 */
function countImages(content) {
  // Match markdown image syntax ![alt text](path)
  const imageRegex = /!\[.*?\]\([^)]+\)/g;
  const matches = content.match(imageRegex);
  return matches ? matches.length : 0;
}

/**
 * Extract code blocks from content
 * @param {string} content - The content to analyze
 * @returns {Array} - Array of code block contents
 */
function extractCodeBlocks(content) {
  const codeBlockRegex = /```(\w+)?\n([\s\S]*?)\n```/g;
  const codeBlocks = [];
  let match;

  while ((match = codeBlockRegex.exec(content)) !== null) {
    codeBlocks.push({
      language: match[1] || 'text',
      code: match[2]
    });
  }

  return codeBlocks;
}

/**
 * Validate a single file
 * @param {string} filePath - Path to the file to validate
 * @returns {Object} - Validation results
 */
function validateFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const results = {
    filePath: filePath,
    errors: [],
    warnings: []
  };

  // Check code blocks
  const codeBlocks = extractCodeBlocks(content);
  codeBlocks.forEach((block, index) => {
    if (isCodeBlockTooLong(block.code)) {
      results.errors.push(
        `Code block ${index + 1} exceeds ${MAX_CODE_LINES} lines (${block.code.split('\n').length} lines)`
      );
    }

    if (!ALLOWED_CODE_LANGUAGES.includes(block.language)) {
      results.errors.push(
        `Code block ${index + 1} uses unsupported language: ${block.language}`
      );
    }
  });

  // Check image count
  const imageCount = countImages(content);
  if (imageCount > MAX_IMAGES_PER_CHAPTER) {
    results.errors.push(
      `File contains ${imageCount} images, maximum allowed is ${MAX_IMAGES_PER_CHAPTER}`
    );
  }

  return results;
}

/**
 * Validate all files in a directory
 * @param {string} dirPath - Directory to validate
 * @returns {Array} - Array of validation results
 */
function validateDirectory(dirPath) {
  const results = [];
  const files = fs.readdirSync(dirPath);

  files.forEach(file => {
    const filePath = path.join(dirPath, file);
    const stat = fs.statSync(filePath);

    if (stat.isDirectory()) {
      results.push(...validateDirectory(filePath));
    } else if (file.endsWith('.md') || file.endsWith('.mdx')) {
      results.push(validateFile(filePath));
    }
  });

  return results;
}

/**
 * Main execution
 */
function main() {
  console.log('Starting content validation...\n');

  const results = validateDirectory(DOCS_DIR);
  const totalFiles = results.length;
  const filesWithErrors = results.filter(r => r.errors.length > 0).length;
  const totalErrors = results.reduce((sum, r) => sum + r.errors.length, 0);
  const totalWarnings = results.reduce((sum, r) => sum + r.warnings.length, 0);

  // Print individual results
  results.forEach(result => {
    if (result.errors.length > 0 || result.warnings.length > 0) {
      console.log(`\n--- ${result.filePath} ---`);
      result.errors.forEach(error => console.log(`ERROR: ${error}`));
      result.warnings.forEach(warning => console.log(`WARNING: ${warning}`));
    }
  });

  // Print summary
  console.log('\n--- Validation Summary ---');
  console.log(`Total files checked: ${totalFiles}`);
  console.log(`Files with errors: ${filesWithErrors}`);
  console.log(`Total errors: ${totalErrors}`);
  console.log(`Total warnings: ${totalWarnings}`);

  if (totalErrors > 0) {
    console.log('\nValidation FAILED: Content does not comply with constitution requirements.');
    process.exit(1);
  } else {
    console.log('\nValidation PASSED: All content complies with constitution requirements.');
    process.exit(0);
  }
}

// Run validation
if (require.main === module) {
  main();
}