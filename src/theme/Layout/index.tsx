import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatComponent from '@site/src/components/ChatComponent';
import TextSelectionHandler from '@site/src/components/TextSelectionHandler';
import { ChatProvider } from '@site/src/contexts/ChatContext';

export default function Layout(props) {
  return (
    <ChatProvider>
      <OriginalLayout {...props} />
      <TextSelectionHandler />
      <ChatComponent />
    </ChatProvider>
  );
}