#!/usr/bin/env node

/**
 * Image Optimization Script for Physical AI & Humanoid Robotics Textbook
 * Optimizes SVG and PNG images for web delivery
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// Configuration
const STATIC_IMG_DIR = './static/img';
const MAX_SVG_SIZE = 500 * 1024; // 500KB
const MAX_PNG_SIZE = 2 * 1024 * 1024; // 2MB

/**
 * Optimize SVG files using SVGO (if available)
 * @param {string} filePath - Path to SVG file
 */
function optimizeSVG(filePath) {
  try {
    // For now, we'll just check the size and report
    const stats = fs.statSync(filePath);
    if (stats.size > MAX_SVG_SIZE) {
      console.log(`WARNING: SVG file ${filePath} is ${Math.round(stats.size/1024)}KB, consider optimization`);
    } else {
      console.log(`OK: SVG file ${filePath} is ${Math.round(stats.size/1024)}KB`);
    }
  } catch (error) {
    console.error(`Error processing SVG ${filePath}: ${error.message}`);
  }
}

/**
 * Optimize PNG files using imagemin (if available)
 * @param {string} filePath - Path to PNG file
 */
function optimizePNG(filePath) {
  try {
    const stats = fs.statSync(filePath);
    if (stats.size > MAX_PNG_SIZE) {
      console.log(`WARNING: PNG file ${filePath} is ${Math.round(stats.size/1024/1024)}MB, consider optimization`);
    } else {
      console.log(`OK: PNG file ${filePath} is ${Math.round(stats.size/1024)}KB`);
    }
  } catch (error) {
    console.error(`Error processing PNG ${filePath}: ${error.message}`);
  }
}

/**
 * Process all images in a directory
 * @param {string} dirPath - Directory to process
 */
function processImagesInDirectory(dirPath) {
  const items = fs.readdirSync(dirPath);

  items.forEach(item => {
    const itemPath = path.join(dirPath, item);
    const stat = fs.statSync(itemPath);

    if (stat.isDirectory()) {
      // Process subdirectories (module-specific image directories)
      processImagesInDirectory(itemPath);
    } else if (item.toLowerCase().endsWith('.svg')) {
      optimizeSVG(itemPath);
    } else if (item.toLowerCase().endsWith('.png')) {
      optimizePNG(itemPath);
    }
  });
}

/**
 * Main execution
 */
function main() {
  console.log('Starting image optimization check...\n');

  if (!fs.existsSync(STATIC_IMG_DIR)) {
    console.log(`Directory ${STATIC_IMG_DIR} does not exist, creating it...`);
    fs.mkdirSync(STATIC_IMG_DIR, { recursive: true });
  }

  processImagesInDirectory(STATIC_IMG_DIR);

  console.log('\nImage optimization check completed.');
  console.log(`Note: This script checks image sizes against limits:`);
  console.log(`- SVG files: max ${MAX_SVG_SIZE/1024}KB`);
  console.log(`- PNG files: max ${MAX_PNG_SIZE/1024/1024}MB`);
  console.log('\nFor actual optimization, consider installing and using:');
  console.log('- svgo for SVG optimization: npm install -g svgo');
  console.log('- imagemin for PNG optimization: npm install imagemin imagemin-pngquant');
}

// Run optimization check
if (require.main === module) {
  main();
}