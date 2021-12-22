import React from 'react';
import useTrack from '../../hooks/useTrack/useTrack';
import AudioTrack from '../AudioTrack/AudioTrack';
import VideoTrack from '../VideoTrack/VideoTrack';

import { IVideoTrack } from '../../types';
import {
  AudioTrack as IAudioTrack,
  LocalTrackPublication,
  Participant,
  RemoteTrackPublication,
  Track,
} from 'twilio-video';
import { createImmediatelyInvokedFunctionExpression } from 'typescript';

interface PublicationProps {
  publication: LocalTrackPublication | RemoteTrackPublication;
  participant: Participant;
  isLocalParticipant?: boolean;
  videoOnly?: boolean;
  videoPriority?: Track.Priority | null;
  rotate?: boolean;
  forceAudio?: boolean;
}

export default function Publication({
  publication,
  isLocalParticipant,
  videoOnly,
  videoPriority,
  rotate,
  forceAudio,
}: //forceAudio
PublicationProps) {
  const track = useTrack(publication);
  const rotateVideo = rotate ? true : false;
  const mustAudio = forceAudio ? true : false;

  if (!track) return null;

  switch (track.kind) {
    case 'video':
      if (mustAudio) {
        // Only render audio for these participants
        return null;
      } else
        return (
          <VideoTrack
            track={track as IVideoTrack}
            priority={videoPriority}
            isLocal={track.name.includes('camera') && isLocalParticipant}
            rotate={rotateVideo}
          />
        );
    case 'audio':
      return videoOnly ? null : <AudioTrack track={track as IAudioTrack} />;
    default:
      return null;
  }
}
