import React from 'react';
import Layout from '@theme-original/Layout';
import ChatPanel from '@site/src/components/ChatPanel';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props}>
        {props.children}
        <ChatPanel />
      </Layout>
    </>
  );
}