import React from 'react';
import Carousel from 'react-material-ui-carousel'

interface CarouselItemProps {
    fileName: string;
}


function Item(props: CarouselItemProps)
{
    return (
        <img style={{ width: '100%' }} src={process.env.PUBLIC_URL + `/images/${props.fileName}`} alt="" />
    )
}

export default function HelpCarousel()
{
    var items = [
        {
            fileName: "tutorial-1.png",
            //description: "Probably the most random thing you have ever seen!"
        },
        {
            fileName: "tutorial-2.png",
            //description: "Hello World!"
        },
        {
            fileName: "tutorial-3.png",
            //description: "Probably the most random thing you have ever seen!"
        }
    ];

    return (
        <Carousel autoPlay={false} navButtonsAlwaysVisible={true}>
            {
                items.map( (item, i) => <Item key={i} fileName={item.fileName} /> )
            }
        </Carousel>
    )
}
