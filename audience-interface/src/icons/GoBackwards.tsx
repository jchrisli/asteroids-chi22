import React from 'react';

export default function GoBackwards() {
  //return <img src={process.env.PUBLIC_URL + '/images/down-arrow.png'} alt="" />;
  return (
    <svg width="64" height="64" viewBox="0 0 64 64" fill="none" xmlns="http://www.w3.org/2000/svg">
      <rect
        x="62"
        y="62"
        width="60"
        height="60"
        rx="9"
        transform="rotate(-180 62 62)"
        stroke="#a4bc13"
        strokeWidth="3"
      />
      <path d="M32.5 16.0312V46.9688" stroke="#a4bc13" strokeWidth="4" strokeLinecap="round" strokeLinejoin="round" />
      <path
        d="M45.1562 34.3125L32.5 46.9688L19.8438 34.3125"
        stroke="#a4bc13"
        strokeWidth="4"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  );
}
