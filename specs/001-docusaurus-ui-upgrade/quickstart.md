# Quickstart: Docusaurus UI Upgrade

## Prerequisites
- Node.js 18+
- npm or yarn
- Git

## Setup
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd D:/Hackathon-1-Book-2025/physical-ai-book
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start development server:
   ```bash
   npm run start
   ```

## Key Files to Modify
- `docusaurus.config.js` - Theme and styling configuration
- `src/css/custom.css` - Custom CSS overrides
- `src/components/` - Custom components (if needed)
- `src/pages/` - Custom pages (if needed)

## Development Workflow
1. Make CSS changes in `src/css/custom.css`
2. Update theme config in `docusaurus.config.js`
3. Test in browser at `http://localhost:3000`
4. Verify all pages render correctly
5. Check responsive behavior on different screen sizes
6. Validate accessibility compliance

## Build and Deploy
```bash
npm run build
```
The built site will be in the `build/` directory and can be deployed to GitHub Pages.