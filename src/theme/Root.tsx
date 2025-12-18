import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot';

// Default implementation wrapping the entire app with the chatbot
export default function Root({ children }: { children: React.ReactNode }): JSX.Element {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
}
