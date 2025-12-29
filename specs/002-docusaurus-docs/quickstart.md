# Quickstart: Docusaurus Documentation Framework

## Prerequisites

- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

## Setup

1. **Navigate to the documentation directory**
   ```bash
   cd physical-ai-book
   ```

2. **Install dependencies**
   ```bash
   npm install
   # or
   yarn install
   ```

3. **Start the development server**
   ```bash
   npm run start
   # or
   yarn start
   ```

4. **Open your browser to** `http://localhost:3000`

## Creating New Documentation

1. **Create a new .md file** in the appropriate module directory under `docs/`
2. **Add front-matter** at the top of the file:
   ```markdown
   ---
   sidebar_label: Your Document Title
   title: Your Document Title
   ---
   ```
3. **Write your content** using standard Markdown syntax
4. **Update sidebars.ts** to include your new document in the navigation

## Building for Production

```bash
npm run build
# or
yarn build
```

The built site will be available in the `build/` directory and can be deployed to any static hosting service.

## Adding to Navigation

To add your document to the sidebar navigation:

1. Open `sidebars.ts`
2. Add your document ID to the appropriate category in the sidebar configuration
3. The document ID is the filename without the `.md` extension

## Documentation Structure

All documentation follows this hierarchy:
```
docs/
├── intro.md
├── module-1/
│   ├── intro.md
│   └── [other module 1 docs]
├── module-2/
│   ├── intro.md
│   └── [other module 2 docs]
├── module-3/
│   ├── intro.md
│   └── [other module 3 docs]
└── module-4/
    ├── intro.md
    └── [other module 4 docs]
```

## Best Practices

- Use clear, descriptive titles for documents
- Include relevant tags for better searchability
- Follow the established module structure
- Use proper front-matter metadata in all documents
- Test documentation locally before committing
