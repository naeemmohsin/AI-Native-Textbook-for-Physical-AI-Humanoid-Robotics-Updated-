/**
 * Root component wrapper for Docusaurus.
 *
 * Wraps the entire app to inject the ChatWidget on all pages.
 * See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

interface Props {
  children: React.ReactNode;
}

export default function Root({ children }: Props): React.ReactElement {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
