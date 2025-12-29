# Data Model: Docusaurus Documentation Framework

## Documentation Content Structure

### Document Entity
- **id**: string - Unique identifier for the document
- **title**: string - Title of the document
- **sidebar_label**: string - Label used in sidebar navigation
- **description**: string (optional) - Brief description of the document content
- **tags**: array of strings - Tags for categorization and search
- **authors**: array of strings - Content authors
- **date**: string - Creation/modification date in YYYY-MM-DD format
- **draft**: boolean - Whether the document is a draft (optional, defaults to false)

### Module Entity
- **name**: string - Name of the curriculum module
- **description**: string - Brief description of the module
- **documents**: array of Document - List of documents in the module
- **order**: number - Order in which the module appears in navigation

### Sidebar Category
- **type**: string - Always "category" for hierarchical organization
- **label**: string - Display name for the category
- **items**: array of (string or Category) - Documents or subcategories in the category

## Front-matter Schema
Each documentation file follows this standard front-matter structure:

```yaml
---
sidebar_label: <display name for sidebar>
title: <document title>
description: <optional description for SEO>
tags: [<optional array of tags>]
authors: [<optional array of author names>]
date: <YYYY-MM-DD format>
draft: <optional boolean, defaults to false>
---
```

## Validation Rules
- All documentation files must be in .md format
- All documentation files must include required front-matter properties
- Document IDs must be unique across the entire documentation set
- Sidebar navigation must follow hierarchical structure without circular references
- Module ordering must be sequential and non-overlapping

## State Transitions
- Draft → Published: When document is ready for public consumption
- Published → Archived: When document is outdated but needs to be retained for historical purposes
