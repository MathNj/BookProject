/**
 * Swizzled Root Component for Docusaurus
 *
 * This wraps the default Root component from @docusaurus/theme-classic
 * and injects the RAG ChatWidget on every page.
 */

import React from 'react';
import RootComponent from '@theme-original/Root';
import ChatWidget from '@site/src/components/ChatWidget';
import type RootType from '@theme/Root';
import type { WrapperProps } from '@docusaurus/types';

type RootProps = WrapperProps<typeof RootType>;

export default function Root(props: RootProps): JSX.Element {
  return (
    <>
      <RootComponent {...props} />
      <ChatWidget />
    </>
  );
}
